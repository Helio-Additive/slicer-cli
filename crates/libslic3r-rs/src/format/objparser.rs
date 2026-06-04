//! OBJ/MTL file parser.
//!
//! C++ Reference:
//! - Format/objparser.hpp
//! - Format/objparser.cpp
//!
//! Provides low-level parsing of Wavefront OBJ and MTL files into structured
//! data that the higher-level `obj` module consumes to build `TriangleMesh` / `Model`.

use std::collections::HashMap;
use std::io::{self, BufRead, Read};
use std::path::Path;
use std::sync::Arc;

use log::error;

// ---------------------------------------------------------------------------
// Constants  (objparser.hpp:93-95)
// ---------------------------------------------------------------------------

/// Number of floats stored per vertex coordinate entry:
/// x, y, z, color_x, color_y, color_z, color_w
pub const OBJ_VERTEX_LENGTH: usize = 7;

/// Index of the alpha channel inside the per-vertex colour block.
pub const OBJ_VERTEX_COLOR_ALPHA: usize = 6;

/// Maximum number of vertex indices in a single face line (quad).
pub const ONE_FACE_SIZE: usize = 4;

// ---------------------------------------------------------------------------
// Data structures  (objparser.hpp)
// ---------------------------------------------------------------------------

/// A single vertex reference inside a face definition.
/// objparser.hpp:12-17
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObjVertex {
    pub coord_idx: i32,
    pub texture_coord_idx: i32,
    pub normal_idx: i32,
}

impl ObjVertex {
    /// Sentinel vertex used to delimit faces in the vertex list.
    pub fn delimiter() -> Self {
        Self {
            coord_idx: -1,
            texture_coord_idx: -1,
            normal_idx: -1,
        }
    }
}

/// `usemtl` directive – maps a material name to a range of vertices/faces.
/// objparser.hpp:26-33
#[derive(Debug, Clone, PartialEq)]
pub struct ObjUseMtl {
    pub vertex_idx_first: i32,
    pub vertex_idx_end: i32,
    pub face_start: i32,
    pub face_end: i32,
    pub name: String,
}

impl ObjUseMtl {
    pub fn new() -> Self {
        Self {
            vertex_idx_first: 0,
            vertex_idx_end: -1,
            face_start: 0,
            face_end: -1,
            name: String::new(),
        }
    }
}

/// Material definition parsed from an MTL file.
/// objparser.hpp:35-49
#[derive(Debug, Clone)]
pub struct ObjNewMtl {
    pub name: String,
    pub ns: f32,
    pub ni: f32,
    pub d: f32,
    pub illum: f32,
    /// Transmission factor (default 1.0).
    pub tr: f32,
    pub tf: [f32; 3],
    pub ka: [f32; 3],
    pub kd: [f32; 3],
    pub ks: [f32; 3],
    pub ke: [f32; 3],
    /// Diffuse texture map filename.
    pub map_kd: String,
}

impl ObjNewMtl {
    pub fn new() -> Self {
        Self {
            name: String::new(),
            ns: 0.0,
            ni: 0.0,
            d: 0.0,
            illum: 0.0,
            tr: 1.0,
            tf: [0.0; 3],
            ka: [0.0; 3],
            kd: [0.0; 3],
            ks: [0.0; 3],
            ke: [0.0; 3],
            map_kd: String::new(),
        }
    }
}

/// Named object (`o` directive).
/// objparser.hpp:57-61
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObjObject {
    pub vertex_idx_first: i32,
    pub name: String,
}

/// Named group (`g` directive).
/// objparser.hpp:70-74
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObjGroup {
    pub vertex_idx_first: i32,
    pub name: String,
}

/// Smoothing group (`s` directive).
/// objparser.hpp:82-86
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObjSmoothingGroup {
    pub vertex_idx_first: i32,
    pub smoothing_group_id: i64,
}

/// All data parsed from a single OBJ file.
/// objparser.hpp:96-122
#[derive(Debug, Clone)]
pub struct ObjData {
    /// Binary format version tag.
    pub version: i32,
    /// Interleaved vertex data: x,y,z, color_x,color_y,color_z,color_w per vertex.
    pub coordinates: Vec<f32>,
    /// Whether any vertex carries colour information.
    pub has_vertex_color: bool,
    /// Texture coordinates: u,v per entry.
    pub texture_coordinates: Vec<f32>,
    /// Vertex normals: x,y,z per entry.
    pub normals: Vec<f32>,
    /// Vertex parameters: u,v,w per entry.
    pub parameters: Vec<f32>,
    /// Referenced MTL library filenames.
    pub mtllibs: Vec<String>,
    /// `usemtl` directives.
    pub usemtls: Vec<ObjUseMtl>,
    /// Named objects.
    pub objects: Vec<ObjObject>,
    /// Named groups.
    pub groups: Vec<ObjGroup>,
    /// Smoothing groups.
    pub smoothing_groups: Vec<ObjSmoothingGroup>,
    /// Face vertex references, delimited by `ObjVertex::delimiter()`.
    pub vertices: Vec<ObjVertex>,

    // MakerLab metadata
    pub ml_region: String,
    pub ml_name: String,
    pub ml_id: String,
}

impl ObjData {
    pub fn new() -> Self {
        Self {
            version: 0,
            coordinates: Vec::new(),
            has_vertex_color: false,
            texture_coordinates: Vec::new(),
            normals: Vec::new(),
            parameters: Vec::new(),
            mtllibs: Vec::new(),
            usemtls: Vec::new(),
            objects: Vec::new(),
            groups: Vec::new(),
            smoothing_groups: Vec::new(),
            vertices: Vec::new(),
            ml_region: String::new(),
            ml_name: String::new(),
            ml_id: String::new(),
        }
    }
}

/// All data parsed from an MTL file.
/// objparser.hpp:124-131
#[derive(Debug, Clone)]
pub struct MtlData {
    pub version: i32,
    pub first_time_using_makerlab: bool,
    pub new_mtl_unmap: HashMap<String, Arc<ObjNewMtl>>,
    pub mtl_orders: Vec<String>,
}

impl MtlData {
    pub fn new() -> Self {
        Self {
            version: 0,
            first_time_using_makerlab: false,
            new_mtl_unmap: HashMap::new(),
            mtl_orders: Vec::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// Internal line parsers  (objparser.cpp)
// ---------------------------------------------------------------------------

/// Eat leading whitespace from a byte slice, returning the remainder.
#[inline]
fn eat_ws(s: &str) -> &str {
    s.trim_start_matches(|c: char| c == ' ' || c == '\t')
}

/// Try to parse a float at the start of `s`. Returns `(value, rest)` on success.
fn parse_float(s: &str) -> Option<(f64, &str)> {
    let s = eat_ws(s);
    // Find the end of the numeric token
    let end = s
        .find(|c: char| c == ' ' || c == '\t' || c == '/')
        .unwrap_or(s.len());
    if end == 0 {
        return None;
    }
    let val: f64 = s[..end].parse().ok()?;
    Some((val, &s[end..]))
}

/// Try to parse an integer at the start of `s`. Returns `(value, rest)` on success.
fn parse_int(s: &str) -> Option<(i64, &str)> {
    let s = eat_ws(s);
    let end = s
        .find(|c: char| c == ' ' || c == '\t' || c == '/')
        .unwrap_or(s.len());
    if end == 0 {
        return None;
    }
    let val: i64 = s[..end].parse().ok()?;
    Some((val, &s[end..]))
}

/// Parse a single OBJ line and accumulate results into `data`.
/// objparser.cpp:12-374
fn obj_parseline(line: &str, data: &mut ObjData) -> bool {
    let line = eat_ws(line);
    if line.is_empty() {
        return true;
    }

    let first = line.as_bytes()[0];
    let rest = &line[1..];

    match first {
        b'#' => {
            // Comment – ignore.
        }
        b'v' => {
            if rest.is_empty() {
                return false;
            }
            match rest.as_bytes()[0] {
                b't' => {
                    // vt – texture coordinate
                    let s = &rest[1..];
                    if !s.starts_with(' ') && !s.starts_with('\t') {
                        return false;
                    }
                    let (u, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let mut v = 0.0;
                    let s = eat_ws(s);
                    if !s.is_empty() {
                        if let Some((val, _)) = parse_float(s) {
                            v = val;
                        }
                    }
                    data.texture_coordinates.push(u as f32);
                    data.texture_coordinates.push(v as f32);
                }
                b'n' => {
                    // vn – vertex normal
                    let s = &rest[1..];
                    if !s.starts_with(' ') && !s.starts_with('\t') {
                        return false;
                    }
                    let (x, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let (y, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let (z, _s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    data.normals.push(x as f32);
                    data.normals.push(y as f32);
                    data.normals.push(z as f32);
                }
                b'p' => {
                    // vp – vertex parameter
                    let s = &rest[1..];
                    if !s.starts_with(' ') && !s.starts_with('\t') {
                        return false;
                    }
                    let (u, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let (v, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let mut w = 0.0;
                    let s = eat_ws(s);
                    if !s.is_empty() {
                        if let Some((val, _)) = parse_float(s) {
                            w = val;
                        }
                    }
                    data.parameters.push(u as f32);
                    data.parameters.push(v as f32);
                    data.parameters.push(w as f32);
                }
                _ => {
                    // v – vertex position (+ optional colour)
                    let s = rest;
                    if !s.starts_with(' ') && !s.starts_with('\t') {
                        return false;
                    }
                    let (x, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let (y, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let (z, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let s = eat_ws(s);
                    let (mut color_x, mut color_y, mut color_z, mut color_w) =
                        (0.0f64, 0.0f64, 0.0f64, 0.0f64);
                    if !s.is_empty() {
                        if !data.has_vertex_color {
                            data.has_vertex_color = true;
                        }
                        let (cx, s) = match parse_float(s) {
                            Some(v) => v,
                            None => return false,
                        };
                        color_x = cx;
                        let (cy, s) = match parse_float(s) {
                            Some(v) => v,
                            None => return false,
                        };
                        color_y = cy;
                        let (cz, s) = match parse_float(s) {
                            Some(v) => v,
                            None => return false,
                        };
                        color_z = cz;
                        color_w = 1.0; // default alpha
                        let s = eat_ws(s);
                        if !s.is_empty() {
                            if let Some((cw, _)) = parse_float(s) {
                                color_w = cw;
                            }
                        }
                    }
                    data.coordinates.push(x as f32);
                    data.coordinates.push(y as f32);
                    data.coordinates.push(z as f32);
                    data.coordinates.push(color_x as f32);
                    data.coordinates.push(color_y as f32);
                    data.coordinates.push(color_z as f32);
                    data.coordinates.push(color_w as f32);
                }
            }
        }
        b'f' => {
            // face
            let mut s = eat_ws(rest);
            if s.is_empty() {
                return false;
            }
            while !s.is_empty() {
                let mut vertex = ObjVertex {
                    coord_idx: 0,
                    texture_coord_idx: 0,
                    normal_idx: 0,
                };
                // Parse coord index
                let (ci, remainder) = match parse_int(s) {
                    Some(v) => v,
                    None => return false,
                };
                vertex.coord_idx = ci as i32;
                s = remainder;

                if s.starts_with('/') {
                    s = &s[1..];
                    if !s.starts_with('/') {
                        // Parse texture coordinate index
                        if let Some((ti, remainder)) = parse_int(s) {
                            vertex.texture_coord_idx = ti as i32;
                            s = remainder;
                        }
                    }
                    if s.starts_with('/') {
                        s = &s[1..];
                        // Parse normal index
                        if let Some((ni, remainder)) = parse_int(s) {
                            vertex.normal_idx = ni as i32;
                            s = remainder;
                        }
                    }
                }

                // Convert relative/1-based indices to 0-based
                if vertex.coord_idx < 0 {
                    vertex.coord_idx += (data.coordinates.len() / OBJ_VERTEX_LENGTH) as i32;
                } else {
                    vertex.coord_idx -= 1;
                }
                if vertex.normal_idx < 0 {
                    vertex.normal_idx += (data.normals.len() / 3) as i32;
                } else {
                    vertex.normal_idx -= 1;
                }
                if vertex.texture_coord_idx < 0 {
                    vertex.texture_coord_idx += (data.texture_coordinates.len() / 3) as i32;
                } else {
                    vertex.texture_coord_idx -= 1;
                }

                data.vertices.push(vertex);
                s = eat_ws(s);
            }

            // Update usemtl face tracking
            if !data.usemtls.is_empty() {
                let last = data.usemtls.last_mut().unwrap();
                last.vertex_idx_end = data.vertices.len() as i32;
            }
            if !data.usemtls.is_empty() {
                // Count face vertices just added (before delimiter)
                let mut face_index_count = 0i32;
                for i in (0..data.vertices.len()).rev() {
                    if data.vertices[i].coord_idx == -1 {
                        break;
                    }
                    face_index_count += 1;
                }
                let last = data.usemtls.last_mut().unwrap();
                if face_index_count == 3 {
                    last.face_end += 1;
                } else if face_index_count == 4 {
                    last.face_end += 2;
                }
            }

            // Push face delimiter
            data.vertices.push(ObjVertex::delimiter());
        }
        b'm' => {
            // mtllib
            if !rest.starts_with("tllib") {
                return false;
            }
            let s = eat_ws(&rest[5..]);
            data.mtllibs.push(s.to_string());
        }
        b'u' => {
            // usemtl
            if !rest.starts_with("semtl") {
                return false;
            }
            let s = eat_ws(&rest[5..]);
            if !data.usemtls.is_empty() {
                let last = data.usemtls.last_mut().unwrap();
                last.vertex_idx_end = data.vertices.len() as i32;
            }
            let mut usemtl = ObjUseMtl::new();
            usemtl.vertex_idx_first = data.vertices.len() as i32;
            usemtl.name = s.to_string();

            if data.usemtls.is_empty() {
                usemtl.face_start = 0;
            } else {
                let count = data.usemtls.len();
                let prev_face_end = data.usemtls[count - 1].face_end;
                usemtl.face_start = prev_face_end + 1;
            }
            usemtl.face_end = usemtl.face_start - 1;
            data.usemtls.push(usemtl);
        }
        b'o' => {
            // o [object name]
            let s = eat_ws(rest);
            // Skip to end of name token
            let name_end = s.find(|c: char| c == ' ' || c == '\t').unwrap_or(s.len());
            let _name = &s[..name_end];
            let obj = ObjObject {
                vertex_idx_first: data.vertices.len() as i32,
                name: s.to_string(),
            };
            data.objects.push(obj);
        }
        b'g' => {
            // g [group name]
            let s = eat_ws(rest);
            let grp = ObjGroup {
                vertex_idx_first: data.vertices.len() as i32,
                name: s.to_string(),
            };
            data.groups.push(grp);
        }
        b's' => {
            // s [smoothing group id]
            let s = rest;
            if !s.starts_with(' ') && !s.starts_with('\t') {
                return false;
            }
            let s = eat_ws(s);
            let g = if s == "off" {
                0i64
            } else {
                match parse_int(s) {
                    Some((v, _)) => v,
                    None => return false,
                }
            };
            data.smoothing_groups.push(ObjSmoothingGroup {
                vertex_idx_first: data.vertices.len() as i32,
                smoothing_group_id: g,
            });
        }
        _ => {
            error!("ObjParser: Unknown command: {}", first as char);
        }
    }
    true
}

/// Parse a single MTL line and accumulate results into `data`.
/// objparser.cpp:376-577
fn mtl_parseline(line: &str, data: &mut MtlData, cur_mtl_name: &mut String) -> bool {
    let line = eat_ws(line);
    if line.is_empty() {
        return true;
    }

    let first = line.as_bytes()[0];
    let rest = &line[1..];

    match first {
        b'#' => {
            // Check for #FirstTimeUsingMakerLab
            if rest.starts_with("FirstTimeUsingMakerLab") {
                data.first_time_using_makerlab = true;
            }
        }
        b'n' => {
            // newmtl
            if !rest.starts_with("ewmtl") {
                return false;
            }
            let s = eat_ws(&rest[5..]);
            *cur_mtl_name = s.to_string();
            data.new_mtl_unmap
                .insert(cur_mtl_name.clone(), Arc::new(ObjNewMtl::new()));
            data.mtl_orders.push(cur_mtl_name.clone());
        }
        b'm' => {
            // map_Kd
            if !rest.starts_with("ap_Kd") {
                return false;
            }
            let s = eat_ws(&rest[5..]);
            if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name) {
                Arc::make_mut(mtl).map_kd = s.to_string();
            }
        }
        b'N' => {
            if rest.is_empty() {
                return false;
            }
            match rest.as_bytes()[0] {
                b's' => {
                    let s = eat_ws(&rest[1..]);
                    if let Some((ns, _)) = parse_float(s) {
                        if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name) {
                            Arc::make_mut(mtl).ns = ns as f32;
                        }
                    }
                }
                b'i' => {
                    let s = eat_ws(&rest[1..]);
                    if let Some((ni, _)) = parse_float(s) {
                        if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name) {
                            Arc::make_mut(mtl).ni = ni as f32;
                        }
                    }
                }
                _ => {}
            }
        }
        b'K' => {
            if rest.is_empty() {
                return false;
            }
            let which = rest.as_bytes()[0];
            let s = eat_ws(&rest[1..]);
            let (x, s) = match parse_float(s) {
                Some(v) => v,
                None => return false,
            };
            let (y, s) = match parse_float(s) {
                Some(v) => v,
                None => return false,
            };
            let (z, _s) = match parse_float(s) {
                Some(v) => v,
                None => return false,
            };
            if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name) {
                let mtl = Arc::make_mut(mtl);
                match which {
                    b'a' => mtl.ka = [x as f32, y as f32, z as f32],
                    b'd' => mtl.kd = [x as f32, y as f32, z as f32],
                    b's' => mtl.ks = [x as f32, y as f32, z as f32],
                    b'e' => mtl.ke = [x as f32, y as f32, z as f32],
                    _ => {}
                }
            }
        }
        b'i' => {
            // illum
            if !rest.starts_with("llum") {
                return false;
            }
            let s = eat_ws(&rest[4..]);
            if let Some((val, _)) = parse_float(s) {
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name) {
                    Arc::make_mut(mtl).illum = val as f32;
                }
            }
        }
        b'd' => {
            let s = eat_ws(rest);
            if let Some((val, _)) = parse_float(s) {
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name) {
                    Arc::make_mut(mtl).d = val as f32;
                }
            }
        }
        b'T' => {
            if rest.is_empty() {
                return false;
            }
            match rest.as_bytes()[0] {
                b'r' => {
                    let s = eat_ws(&rest[1..]);
                    if let Some((val, _)) = parse_float(s) {
                        if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name) {
                            let tr = if val > 0.0 && val <= 1.0 {
                                val as f32
                            } else {
                                1.0
                            };
                            Arc::make_mut(mtl).tr = tr;
                        }
                    }
                }
                b'f' => {
                    let s = eat_ws(&rest[1..]);
                    let (x, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let (y, s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    let (z, _s) = match parse_float(s) {
                        Some(v) => v,
                        None => return false,
                    };
                    if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name) {
                        Arc::make_mut(mtl).tf = [x as f32, y as f32, z as f32];
                    }
                }
                _ => {}
            }
        }
        _ => {
            // Unknown MTL directive – ignore silently (matches C++ behaviour).
        }
    }
    true
}

// ---------------------------------------------------------------------------
// Public API  (objparser.hpp:132-142)
// ---------------------------------------------------------------------------

/// Parse an OBJ file from a filesystem path.
/// objparser.cpp:579-628
pub fn objparse(path: &Path, data: &mut ObjData) -> bool {
    let contents = match std::fs::read(path) {
        Ok(c) => c,
        Err(_) => return false,
    };

    let mut line_count: usize = 0;

    for raw_line in contents.split(|&b| b == b'\n' || b == b'\r') {
        if raw_line.is_empty() {
            continue;
        }
        let line_str = String::from_utf8_lossy(raw_line);
        let trimmed = eat_ws(&line_str);

        obj_parseline(trimmed, data);

        // MakerLab metadata from the first 3 lines
        if line_count == 0 {
            data.ml_region = parsemlinfo(trimmed, "region:");
        }
        if line_count == 1 {
            data.ml_name = parsemlinfo(trimmed, "ml_name:");
        }
        if line_count == 2 {
            data.ml_id = parsemlinfo(trimmed, "ml_file_id:");
        }

        line_count += 1;
    }
    true
}

/// Parse an OBJ from a generic reader (in-memory stream equivalent).
/// objparser.cpp:691-729
pub fn objparse_stream<R: Read>(reader: R, data: &mut ObjData) -> bool {
    let buf_reader = io::BufReader::new(reader);
    let mut line_count: usize = 0;

    for line_result in buf_reader.lines() {
        let line_str = match line_result {
            Ok(l) => l,
            Err(_) => return false,
        };
        let trimmed = eat_ws(&line_str);
        obj_parseline(trimmed, data);

        if line_count < 3 {
            data.ml_region = parsemlinfo(trimmed, "region");
            data.ml_name = parsemlinfo(trimmed, "ml_name");
            data.ml_id = parsemlinfo(trimmed, "ml_file_id");
        }
        line_count += 1;
    }
    true
}

/// Parse an MTL file from a filesystem path.
/// objparser.cpp:652-689
pub fn mtlparse(path: &Path, data: &mut MtlData) -> bool {
    let contents = match std::fs::read(path) {
        Ok(c) => c,
        Err(_) => return false,
    };

    let mut cur_mtl_name = String::new();

    for raw_line in contents.split(|&b| b == b'\n' || b == b'\r') {
        if raw_line.is_empty() {
            continue;
        }
        let line_str = String::from_utf8_lossy(raw_line);
        let trimmed = eat_ws(&line_str);
        mtl_parseline(trimmed, data, &mut cur_mtl_name);
    }
    true
}

/// Extract MakerLab info from a comment line.
/// objparser.cpp:630-649
pub fn parsemlinfo(input: &str, condition: &str) -> String {
    if let Some(pos) = input.find(condition) {
        let after = &input[pos + condition.len()..];
        let trimmed = eat_ws(after);
        // Take until newline (or end)
        let end = trimmed.find('\n').unwrap_or(trimmed.len());
        trimmed[..end].to_string()
    } else {
        String::new()
    }
}

// ---------------------------------------------------------------------------
// Binary save / load  (objparser.cpp:731-880)
// ---------------------------------------------------------------------------

/// Load OBJ data from a proprietary binary cache file.
/// objparser.cpp:849-880
pub fn objbinload(path: &Path, data: &mut ObjData) -> bool {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(_) => return false,
    };
    let mut cursor = io::Cursor::new(&bytes);

    // Read version
    let version = match read_usize(&mut cursor) {
        Some(v) => v,
        None => return false,
    };
    if version != 1 {
        return false;
    }
    data.version = version as i32;

    load_vec_f32(&mut cursor, &mut data.coordinates)
        && load_vec_f32(&mut cursor, &mut data.texture_coordinates)
        && load_vec_f32(&mut cursor, &mut data.normals)
        && load_vec_f32(&mut cursor, &mut data.parameters)
        && load_vec_string(&mut cursor, &mut data.mtllibs)
        && load_vec_usemtl(&mut cursor, &mut data.usemtls)
        && load_vec_object(&mut cursor, &mut data.objects)
        && load_vec_group(&mut cursor, &mut data.groups)
        && load_vec_smoothing(&mut cursor, &mut data.smoothing_groups)
        && load_vec_vertex(&mut cursor, &mut data.vertices)
}

/// Compare two ObjData structures for equality.
/// objparser.cpp:903-918
pub fn objequal(data1: &ObjData, data2: &ObjData) -> bool {
    data1.coordinates == data2.coordinates
        && data1.texture_coordinates == data2.texture_coordinates
        && data1.normals == data2.normals
        && data1.parameters == data2.parameters
        && data1.mtllibs == data2.mtllibs
        && data1.usemtls == data2.usemtls
        && data1.objects == data2.objects
        && data1.groups == data2.groups
        && data1.vertices == data2.vertices
}

// ---------------------------------------------------------------------------
// Binary helper readers (private)
// ---------------------------------------------------------------------------

fn read_usize<R: Read>(r: &mut R) -> Option<usize> {
    let mut buf = [0u8; std::mem::size_of::<usize>()];
    r.read_exact(&mut buf).ok()?;
    Some(usize::from_ne_bytes(buf))
}

fn read_i32<R: Read>(r: &mut R) -> Option<i32> {
    let mut buf = [0u8; 4];
    r.read_exact(&mut buf).ok()?;
    Some(i32::from_ne_bytes(buf))
}

fn load_vec_f32<R: Read>(r: &mut R, v: &mut Vec<f32>) -> bool {
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    v.clear();
    if cnt > 0 {
        v.resize(cnt, 0.0);
        let byte_len = cnt * std::mem::size_of::<f32>();
        let slice = unsafe { std::slice::from_raw_parts_mut(v.as_mut_ptr() as *mut u8, byte_len) };
        if r.read_exact(slice).is_err() {
            return false;
        }
    }
    true
}

fn load_vec_string<R: Read>(r: &mut R, v: &mut Vec<String>) -> bool {
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    v.clear();
    v.reserve(cnt);
    for _ in 0..cnt {
        let len = match read_usize(r) {
            Some(l) => l,
            None => return false,
        };
        let mut buf = vec![0u8; len];
        if r.read_exact(&mut buf).is_err() {
            return false;
        }
        v.push(String::from_utf8_lossy(&buf).to_string());
    }
    true
}

fn load_vec_usemtl<R: Read>(r: &mut R, v: &mut Vec<ObjUseMtl>) -> bool {
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    v.clear();
    for _ in 0..cnt {
        let idx = match read_i32(r) {
            Some(i) => i,
            None => return false,
        };
        let len = match read_usize(r) {
            Some(l) => l,
            None => return false,
        };
        let mut buf = vec![0u8; len];
        if r.read_exact(&mut buf).is_err() {
            return false;
        }
        let mut um = ObjUseMtl::new();
        um.vertex_idx_first = idx;
        um.name = String::from_utf8_lossy(&buf).to_string();
        v.push(um);
    }
    true
}

fn load_vec_object<R: Read>(r: &mut R, v: &mut Vec<ObjObject>) -> bool {
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    v.clear();
    for _ in 0..cnt {
        let idx = match read_i32(r) {
            Some(i) => i,
            None => return false,
        };
        let len = match read_usize(r) {
            Some(l) => l,
            None => return false,
        };
        let mut buf = vec![0u8; len];
        if r.read_exact(&mut buf).is_err() {
            return false;
        }
        v.push(ObjObject {
            vertex_idx_first: idx,
            name: String::from_utf8_lossy(&buf).to_string(),
        });
    }
    true
}

fn load_vec_group<R: Read>(r: &mut R, v: &mut Vec<ObjGroup>) -> bool {
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    v.clear();
    for _ in 0..cnt {
        let idx = match read_i32(r) {
            Some(i) => i,
            None => return false,
        };
        let len = match read_usize(r) {
            Some(l) => l,
            None => return false,
        };
        let mut buf = vec![0u8; len];
        if r.read_exact(&mut buf).is_err() {
            return false;
        }
        v.push(ObjGroup {
            vertex_idx_first: idx,
            name: String::from_utf8_lossy(&buf).to_string(),
        });
    }
    true
}

fn load_vec_smoothing<R: Read>(r: &mut R, v: &mut Vec<ObjSmoothingGroup>) -> bool {
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    v.clear();
    if cnt > 0 {
        v.resize(
            cnt,
            ObjSmoothingGroup {
                vertex_idx_first: 0,
                smoothing_group_id: 0,
            },
        );
        let byte_len = cnt * std::mem::size_of::<ObjSmoothingGroup>();
        let slice = unsafe { std::slice::from_raw_parts_mut(v.as_mut_ptr() as *mut u8, byte_len) };
        if r.read_exact(slice).is_err() {
            return false;
        }
    }
    true
}

fn load_vec_vertex<R: Read>(r: &mut R, v: &mut Vec<ObjVertex>) -> bool {
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    v.clear();
    if cnt > 0 {
        v.resize(cnt, ObjVertex::delimiter());
        let byte_len = cnt * std::mem::size_of::<ObjVertex>();
        let slice = unsafe { std::slice::from_raw_parts_mut(v.as_mut_ptr() as *mut u8, byte_len) };
        if r.read_exact(slice).is_err() {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parsemlinfo() {
        assert_eq!(parsemlinfo("# region: foo", "region:"), "foo");
        assert_eq!(parsemlinfo("# nothing here", "region:"), "");
    }

    #[test]
    fn test_obj_vertex_delimiter() {
        let d = ObjVertex::delimiter();
        assert_eq!(d.coord_idx, -1);
        assert_eq!(d.texture_coord_idx, -1);
        assert_eq!(d.normal_idx, -1);
    }

    #[test]
    fn test_obj_parseline_vertex() {
        let mut data = ObjData::new();
        assert!(obj_parseline("v 1.0 2.0 3.0", &mut data));
        assert_eq!(data.coordinates.len(), OBJ_VERTEX_LENGTH);
        assert!((data.coordinates[0] - 1.0).abs() < 1e-6);
        assert!((data.coordinates[1] - 2.0).abs() < 1e-6);
        assert!((data.coordinates[2] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_obj_parseline_face() {
        let mut data = ObjData::new();
        // Add 3 vertices first
        obj_parseline("v 0 0 0", &mut data);
        obj_parseline("v 1 0 0", &mut data);
        obj_parseline("v 0 1 0", &mut data);
        obj_parseline("f 1 2 3", &mut data);
        // 3 vertex refs + 1 delimiter
        assert_eq!(data.vertices.len(), 4);
        assert_eq!(data.vertices[3], ObjVertex::delimiter());
    }

    #[test]
    fn test_mtl_parseline_newmtl() {
        let mut data = MtlData::new();
        let mut name = String::new();
        mtl_parseline("newmtl TestMaterial", &mut data, &mut name);
        assert_eq!(name, "TestMaterial");
        assert!(data.new_mtl_unmap.contains_key("TestMaterial"));
    }

    #[test]
    fn test_objequal_identical() {
        let d1 = ObjData::new();
        let d2 = ObjData::new();
        assert!(objequal(&d1, &d2));
    }
}
