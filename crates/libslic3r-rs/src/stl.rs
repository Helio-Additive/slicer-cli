//! 1:1 port of `libslic3r/Format/STL.{hpp,cpp}` (BambuStudio).
//!
//! C++ Reference:
//! - Format/STL.hpp
//! - Format/STL.cpp
//!
//! STL.cpp is a thin wrapper: it delegates the actual file parsing to
//! `TriangleMesh::ReadSTLFile` (which in turn calls admesh `stl_open` /
//! `TriangleMesh::from_stl`) and the writing to `TriangleMesh::write_binary` /
//! `TriangleMesh::write_ascii` (which call `its_write_stl_binary` /
//! `its_write_stl_ascii`). Those mesh methods are NOT yet ported as members of
//! the Rust `TriangleMesh` wrapper, so:
//!   * the STL *reader* is provided here as the free function `read_stl_file`,
//!     standing in for `TriangleMesh::ReadSTLFile` (STL.cpp:22) until that
//!     member is ported. It mirrors admesh's binary/ASCII auto-detection.
//!   * the STL *writers* delegate to the already-ported free functions
//!     `its_write_stl_ascii` / `its_write_stl_binary` in `triangle_mesh`,
//!     which are faithful translations of `TriangleMesh.cpp:1959-2020`.
//!
//! See the `divergences` notes in the port report for the parts of
//! `Model::add_object(name, path, mesh)` (ModelVolume / input_file / source /
//! default-extruder bookkeeping) that are not modelled by the Rust `Model`.

use crate::geometry::Point3F;
use crate::model::{Model, ModelObject};
use crate::normal_utils::{StlTriangleVertexIndices, StlVertex};
use crate::triangle_mesh::{its_write_stl_ascii, its_write_stl_binary, Triangle, TriangleMesh};
use crate::{Error, Result};

use std::collections::HashMap;
use std::path::Path;

// STL.cpp:9-13
#[cfg(target_os = "windows")]
const DIR_SEPARATOR: char = '\\';
#[cfg(not(target_os = "windows"))]
const DIR_SEPARATOR: char = '/';

/// C++ `ImportstlProgressFn` (admesh/stl.h:48-49):
/// `std::function<void(int current, int total, bool& cancel, std::string& model_id,
///  std::string& code, std::string& ml_region, std::string& ml_name, std::string& ml_id)>`.
///
/// The progress callback receives `(current, total)`; the remaining out-params
/// (`cancel` plus the model-id / code / ml_* strings) are threaded through the
/// admesh reader. They are not consumed by the Rust reader yet, so the type is
/// kept for signature parity at the `load_stl` boundary.
pub type ImportStlProgressFn = Box<dyn Fn(i32, i32) -> bool>;

// ---------------------------------------------------------------------------
// Binary STL constants (admesh layout)
// ---------------------------------------------------------------------------

const STL_HEADER_SIZE: usize = 80;
const STL_FACET_SIZE: usize = 50; // normal (12) + 3 vertices (36) + attribute (2)

// ---------------------------------------------------------------------------
// TriangleMesh::ReadSTLFile  (STL.cpp:22 -> TriangleMesh.cpp:215 -> stl_open)
// ---------------------------------------------------------------------------

/// Load an STL file (binary or ASCII) into a `TriangleMesh`.
///
/// Stands in for C++ `TriangleMesh::ReadSTLFile(input_file, repair=true, stlFn,
/// custom_header_length=80)` (TriangleMesh.cpp:215-221), which delegates to
/// admesh `stl_open` and `from_stl`. Returns `Err` on a read failure, matching
/// the C++ `false` return that `load_stl` turns into a `false` result.
pub fn read_stl_file(path: &Path) -> Result<TriangleMesh> {
    let data =
        std::fs::read(path).map_err(|e| Error::IO(format!("Failed to read STL file: {}", e)))?;

    if data.len() < STL_HEADER_SIZE + 4 {
        // Too small for binary; try ASCII.
        return read_stl_ascii(&data);
    }

    // Heuristic: if the file starts with "solid" and is not binary-plausible, treat as ASCII.
    let maybe_ascii = data.starts_with(b"solid") && !is_binary_stl(&data);

    if maybe_ascii {
        read_stl_ascii(&data)
    } else {
        read_stl_binary(&data)
    }
}

/// Quick heuristic: a binary STL declares a facet count in bytes 80..84.
/// If the file size matches `80 + 4 + num_facets * 50`, it is binary.
fn is_binary_stl(data: &[u8]) -> bool {
    if data.len() < STL_HEADER_SIZE + 4 {
        return false;
    }
    let num_facets = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
    let expected = STL_HEADER_SIZE + 4 + num_facets * STL_FACET_SIZE;
    data.len() >= expected
}

/// Parse a binary STL from raw bytes.
fn read_stl_binary(data: &[u8]) -> Result<TriangleMesh> {
    if data.len() < STL_HEADER_SIZE + 4 {
        return Err(Error::Mesh("Binary STL too short".into()));
    }
    let num_facets = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;

    let body = &data[STL_HEADER_SIZE + 4..];
    if body.len() < num_facets * STL_FACET_SIZE {
        return Err(Error::Mesh("Binary STL truncated".into()));
    }

    // We store unique vertices via a simple hash-dedup approach.
    let mut vertices: Vec<Point3F> = Vec::new();
    let mut indices: Vec<Triangle> = Vec::with_capacity(num_facets);
    let mut vertex_map: HashMap<[u32; 3], u32> = HashMap::new();

    for i in 0..num_facets {
        let offset = i * STL_FACET_SIZE;
        // Skip normal (12 bytes), read 3 vertices (each 12 bytes)
        let mut tri_idx = [0u32; 3];
        for v in 0..3 {
            let vo = offset + 12 + v * 12;
            let x = f32::from_le_bytes([body[vo], body[vo + 1], body[vo + 2], body[vo + 3]]);
            let y = f32::from_le_bytes([body[vo + 4], body[vo + 5], body[vo + 6], body[vo + 7]]);
            let z = f32::from_le_bytes([body[vo + 8], body[vo + 9], body[vo + 10], body[vo + 11]]);
            let key = [x.to_bits(), y.to_bits(), z.to_bits()];
            let idx = match vertex_map.get(&key) {
                Some(&idx) => idx,
                None => {
                    let idx = vertices.len() as u32;
                    vertices.push(Point3F::new(x as f64, y as f64, z as f64));
                    vertex_map.insert(key, idx);
                    idx
                }
            };
            tri_idx[v] = idx;
        }
        indices.push(Triangle::new(tri_idx[0], tri_idx[1], tri_idx[2]));
    }

    Ok(TriangleMesh::from_parts(vertices, indices))
}

/// Parse an ASCII STL from raw bytes.
fn read_stl_ascii(data: &[u8]) -> Result<TriangleMesh> {
    let text = String::from_utf8_lossy(data);
    let mut vertices: Vec<Point3F> = Vec::new();
    let mut indices: Vec<Triangle> = Vec::new();
    let mut vertex_map: HashMap<[u32; 3], u32> = HashMap::new();
    let mut face_verts: Vec<u32> = Vec::new();

    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("vertex") {
            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.len() >= 4 {
                let x: f32 = parts[1].parse().unwrap_or(0.0);
                let y: f32 = parts[2].parse().unwrap_or(0.0);
                let z: f32 = parts[3].parse().unwrap_or(0.0);
                let key = [x.to_bits(), y.to_bits(), z.to_bits()];
                let idx = match vertex_map.get(&key) {
                    Some(&idx) => idx,
                    None => {
                        let idx = vertices.len() as u32;
                        vertices.push(Point3F::new(x as f64, y as f64, z as f64));
                        vertex_map.insert(key, idx);
                        idx
                    }
                };
                face_verts.push(idx);
            }
        } else if trimmed.starts_with("endfacet") {
            if face_verts.len() == 3 {
                indices.push(Triangle::new(face_verts[0], face_verts[1], face_verts[2]));
            }
            face_verts.clear();
        }
    }

    if vertices.is_empty() || indices.is_empty() {
        return Err(Error::Mesh("ASCII STL contains no geometry".into()));
    }

    Ok(TriangleMesh::from_parts(vertices, indices))
}

// ---------------------------------------------------------------------------
// load_stl  (STL.cpp:17-40)
// ---------------------------------------------------------------------------

/// Load an STL file into a provided model.
///
/// STL.cpp:17 — `bool load_stl(const char *path, Model *model, const char
/// *object_name_in, ImportstlProgressFn stlFn, int custom_header_length)`.
///
/// `object_name` corresponds to the C++ `object_name_in` (default `nullptr`),
/// `_stl_fn` to `stlFn` (default `nullptr`) and `_custom_header_length` to
/// `custom_header_length` (default `80`). Returns `false` if the file could not
/// be read or the mesh is empty.
pub fn load_stl(
    path: &Path,
    model: &mut Model,
    object_name: Option<&str>,
    _stl_fn: Option<ImportStlProgressFn>,
    _custom_header_length: i32,
) -> bool {
    // STL.cpp:19
    //   TriangleMesh mesh;
    //   std::string design_id;
    let design_id = String::new();
    let _ = design_id;

    // STL.cpp:22-25
    //   if (!mesh.ReadSTLFile(path, true, stlFn, custom_header_length)) {
    //       return false;
    //   }
    let mesh = match read_stl_file(path) {
        Ok(mesh) => mesh,
        Err(_) => return false,
    };
    // STL.cpp:26-29
    //   if (mesh.empty()) {
    //       return false;
    //   }
    if mesh.is_empty() {
        return false;
    }

    // STL.cpp:31-36
    //   std::string object_name;
    //   if (object_name_in == nullptr) {
    //       const char *last_slash = strrchr(path, DIR_SEPARATOR);
    //       object_name.assign((last_slash == nullptr) ? path : last_slash + 1);
    //   } else
    //      object_name.assign(object_name_in);
    let object_name: String = match object_name {
        None => {
            let path_str = path.to_string_lossy();
            match path_str.rfind(DIR_SEPARATOR) {
                None => path_str.to_string(),
                Some(pos) => path_str[pos + 1..].to_string(),
            }
        }
        Some(name) => name.to_string(),
    };

    // STL.cpp:38
    //   model->add_object(object_name.c_str(), path, std::move(mesh));
    //
    // The Rust `Model::add_object` takes a fully-built `ModelObject`. The C++
    // `Model::add_object(name, path, &&mesh)` overload (Model.cpp:485) also sets
    // `input_file`, builds a `ModelVolume` with a `source`, and forces the
    // extruder to 1 — bookkeeping the Rust `Model`/`ModelObject` do not yet
    // model. We faithfully set the object name and mesh.
    let object = ModelObject::new(object_name, mesh);
    model.add_object(object);

    // STL.cpp:39
    //   return true;
    true
}

// ---------------------------------------------------------------------------
// store_stl  (STL.cpp:42-62)
// ---------------------------------------------------------------------------

/// STL.cpp:42 — `bool store_stl(const char *path, TriangleMesh *mesh, bool binary)`.
pub fn store_stl(path: &Path, mesh: &TriangleMesh, binary: bool) -> bool {
    // STL.cpp:44-47
    //   if (binary)
    //       mesh->write_binary(path);
    //   else
    //       mesh->write_ascii(path);
    if binary {
        write_binary(mesh, path);
    } else {
        write_ascii(mesh, path);
    }
    // STL.cpp:48-49
    //   //FIXME returning false even if write failed.
    //   return true;
    true
}

/// STL.cpp:52 — `bool store_stl(const char *path, ModelObject *model_object, bool binary)`.
pub fn store_stl_model_object(path: &Path, model_object: &ModelObject, binary: bool) -> bool {
    // STL.cpp:54
    //   TriangleMesh mesh = model_object->mesh();
    let mesh = model_object.mesh.clone();
    // STL.cpp:55
    //   return store_stl(path, &mesh, binary);
    store_stl(path, &mesh, binary)
}

/// STL.cpp:58 — `bool store_stl(const char *path, Model *model, bool binary)`.
pub fn store_stl_model(path: &Path, model: &Model, binary: bool) -> bool {
    // STL.cpp:60
    //   TriangleMesh mesh = model->mesh();
    //
    // `Model::mesh()` (which merges all objects' meshes) is not yet ported; we
    // merge here, mirroring the merge semantics.
    let mesh = model_mesh(model);
    // STL.cpp:61
    //   return store_stl(path, &mesh, binary);
    store_stl(path, &mesh, binary)
}

// ---------------------------------------------------------------------------
// TriangleMesh::write_binary / write_ascii (TriangleMesh.cpp:223-231)
// ---------------------------------------------------------------------------
//
// These mirror `TriangleMesh::write_ascii`/`write_binary`, which call
// `its_write_stl_ascii`/`its_write_stl_binary(file, "", this->its)`. The Rust
// `TriangleMesh` wrapper stores vertices as `Point3F` (f64) and triangles as
// `Triangle`; we convert to the `indexed_triangle_set` representation
// (`StlVertex` = Vec3f, `StlTriangleVertexIndices` = Vec3i) those writers take,
// then forward with an empty label, exactly as the C++ members do.

/// TriangleMesh.cpp:228-231 — `mesh->write_binary(path)`.
fn write_binary(mesh: &TriangleMesh, path: &Path) -> bool {
    let (indices, vertices) = mesh_to_its(mesh);
    its_write_stl_binary(&path.to_string_lossy(), "", &indices, &vertices)
}

/// TriangleMesh.cpp:223-226 — `mesh->write_ascii(path)`.
fn write_ascii(mesh: &TriangleMesh, path: &Path) -> bool {
    let (indices, vertices) = mesh_to_its(mesh);
    its_write_stl_ascii(&path.to_string_lossy(), "", &indices, &vertices)
}

/// Convert the wrapper `TriangleMesh` into the `indexed_triangle_set`
/// representation (`stl_triangle_vertex_indices` + `stl_vertex`) consumed by
/// `its_write_stl_*`.
fn mesh_to_its(mesh: &TriangleMesh) -> (Vec<StlTriangleVertexIndices>, Vec<StlVertex>) {
    let vertices: Vec<StlVertex> = mesh
        .vertices()
        .iter()
        .map(|v| StlVertex::new(v.x() as f32, v.y() as f32, v.z() as f32))
        .collect();
    let indices: Vec<StlTriangleVertexIndices> = mesh
        .indices()
        .iter()
        .map(|t| {
            StlTriangleVertexIndices::new(
                t.indices[0] as i32,
                t.indices[1] as i32,
                t.indices[2] as i32,
            )
        })
        .collect();
    (indices, vertices)
}

/// Merge all objects' meshes, mirroring C++ `Model::mesh()` (a single combined
/// `TriangleMesh`). Vertex indices are offset per object.
fn model_mesh(model: &Model) -> TriangleMesh {
    let mut all_verts: Vec<Point3F> = Vec::new();
    let mut all_tris: Vec<Triangle> = Vec::new();
    for obj in &model.objects {
        let offset = all_verts.len() as u32;
        all_verts.extend_from_slice(obj.mesh.vertices());
        for tri in obj.mesh.indices() {
            all_tris.push(Triangle::new(
                tri.indices[0] + offset,
                tri.indices[1] + offset,
                tri.indices[2] + offset,
            ));
        }
    }
    TriangleMesh::from_parts(all_verts, all_tris)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_binary_stl_too_short() {
        let data = vec![0u8; 50];
        assert!(!is_binary_stl(&data));
    }

    #[test]
    fn test_roundtrip_binary_stl() {
        let v0 = Point3F::new(0.0, 0.0, 0.0);
        let v1 = Point3F::new(1.0, 0.0, 0.0);
        let v2 = Point3F::new(0.0, 1.0, 0.0);
        let mesh = TriangleMesh::from_parts(vec![v0, v1, v2], vec![Triangle::new(0, 1, 2)]);

        let dir = std::env::temp_dir().join("test_stl_roundtrip");
        std::fs::create_dir_all(&dir).unwrap();
        let file_path = dir.join("test.stl");

        assert!(store_stl(&file_path, &mesh, true));
        let loaded = read_stl_file(&file_path).unwrap();
        assert_eq!(loaded.vertex_count(), 3);
        assert_eq!(loaded.indices().len(), 1);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn test_roundtrip_ascii_stl() {
        let v0 = Point3F::new(0.0, 0.0, 0.0);
        let v1 = Point3F::new(1.0, 0.0, 0.0);
        let v2 = Point3F::new(0.0, 1.0, 0.0);
        let mesh = TriangleMesh::from_parts(vec![v0, v1, v2], vec![Triangle::new(0, 1, 2)]);

        let dir = std::env::temp_dir().join("test_stl_ascii_roundtrip");
        std::fs::create_dir_all(&dir).unwrap();
        let file_path = dir.join("test_ascii.stl");

        assert!(store_stl(&file_path, &mesh, false));
        let loaded = read_stl_file(&file_path).unwrap();
        assert_eq!(loaded.vertex_count(), 3);
        assert_eq!(loaded.indices().len(), 1);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn test_load_stl_into_model() {
        // STL.cpp:17-40 — fills a Model with one object named after the file.
        let v0 = Point3F::new(0.0, 0.0, 0.0);
        let v1 = Point3F::new(1.0, 0.0, 0.0);
        let v2 = Point3F::new(0.0, 1.0, 0.0);
        let mesh = TriangleMesh::from_parts(vec![v0, v1, v2], vec![Triangle::new(0, 1, 2)]);

        let dir = std::env::temp_dir().join("test_stl_load_model");
        std::fs::create_dir_all(&dir).unwrap();
        let file_path = dir.join("part.stl");
        assert!(store_stl(&file_path, &mesh, true));

        let mut model = Model::new();
        assert!(load_stl(&file_path, &mut model, None, None, 80));
        assert_eq!(model.objects.len(), 1);
        // STL.cpp:33-34 — name from basename when object_name_in == nullptr.
        assert_eq!(model.objects[0].name, "part.stl");

        std::fs::remove_dir_all(&dir).ok();
    }
}
