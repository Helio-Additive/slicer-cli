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
//!     member is ported. Its binary/ASCII auto-detection and facet counting
//!     faithfully mirror admesh `stl_open_count_facets` (stlinit.cpp:53-152):
//!     the file is binary iff any byte in the 128-byte window past the header
//!     has its high bit set, and a binary file must have a size that is a whole
//!     number of 50-byte facets and at least `STL_MIN_FILE_SIZE` (284) bytes.
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
// Binary STL constants (admesh layout, admesh/stl.h:34-74)
// ---------------------------------------------------------------------------

// admesh/stl.h:34 — `#define LABEL_SIZE 80`
const LABEL_SIZE: usize = 80;
// admesh/stl.h:36 — `#define NUM_FACET_SIZE 4`
const NUM_FACET_SIZE: usize = 4;
// admesh/stl.h:39 — `#define STL_MIN_FILE_SIZE 284`
const STL_MIN_FILE_SIZE: usize = 284;
// admesh/stl.h:74 — `#define SIZEOF_STL_FACET 50` (normal 12 + 3 vertices 36 + attribute 2)
const SIZEOF_STL_FACET: usize = 50;
// Default `custom_header_length` (STL.hpp:13, admesh uses LABEL_SIZE elsewhere).
const DEFAULT_CUSTOM_HEADER_LENGTH: usize = LABEL_SIZE;

// ---------------------------------------------------------------------------
// TriangleMesh::ReadSTLFile  (STL.cpp:22 -> TriangleMesh.cpp:215 -> stl_open)
// ---------------------------------------------------------------------------

/// Load an STL file (binary or ASCII) into a `TriangleMesh`.
///
/// Stands in for C++ `TriangleMesh::ReadSTLFile(input_file, repair=true, stlFn,
/// custom_header_length=80)` (TriangleMesh.cpp:215-221), which delegates to
/// admesh `stl_open` and `from_stl`. Returns `Err` on a read failure, matching
/// the C++ `false` return that `load_stl` turns into a `false` result.
///
/// FIDELITY-NOTE (out-of-file, TriangleMesh.cpp:215 / admesh): the binary/ASCII
/// detection and facet-count logic below faithfully mirror admesh
/// `stl_open_count_facets` (stlinit.cpp:53-152). The remaining `from_stl`
/// behaviour — `repair=true` mesh repair (fix-normals, remove-degenerate,
/// fill-holes) and admesh's tolerance-based shared-vertex generation
/// (`stl_generate_shared_vertices`) — is NOT reproduced here; this reader
/// dedups vertices by exact f32 bit pattern. Those belong to TriangleMesh.cpp /
/// admesh and are ported there, not in STL.cpp.
pub fn read_stl_file(path: &Path) -> Result<TriangleMesh> {
    read_stl_file_with_header_length(path, DEFAULT_CUSTOM_HEADER_LENGTH as i32)
}

/// Same as [`read_stl_file`] but with an explicit `custom_header_length`,
/// mirroring admesh `stl_open_count_facets`'s `custom_header_length` parameter
/// (stlinit.cpp:53) that `TriangleMesh::ReadSTLFile` (TriangleMesh.cpp:215)
/// threads through.
pub fn read_stl_file_with_header_length(
    path: &Path,
    custom_header_length: i32,
) -> Result<TriangleMesh> {
    let data =
        std::fs::read(path).map_err(|e| Error::IO(format!("Failed to read STL file: {}", e)))?;

    // admesh/stlinit.cpp:66 — `int header_size = custom_header_length + NUM_FACET_SIZE;`
    let header_size = custom_header_length.max(0) as usize + NUM_FACET_SIZE;

    // admesh/stlinit.cpp:67-73 — seek to `header_size`, then
    // `fread(chtest, sizeof(chtest)/*=128*/, 1, fp)`. fread returns the number
    // of *complete* 128-byte blocks read, so unless a full 128-byte window is
    // available past the header, it returns 0 and the file is rejected as empty.
    const CHTEST_SIZE: usize = 128;
    let window = match data.get(header_size..header_size + CHTEST_SIZE) {
        Some(w) => w,
        None => {
            return Err(Error::Mesh(
                "stl_open_count_facets: The input is an empty file".into(),
            ))
        }
    };

    // admesh/stlinit.cpp:74-80 — default to ASCII; classify as binary if any of
    // the 128 bytes past the header has the high bit set (> 127).
    let is_binary = window.iter().any(|&b| b > 127);

    if is_binary {
        read_stl_binary(&data, header_size)
    } else {
        read_stl_ascii(&data)
    }
}

/// Parse a binary STL from raw bytes.
///
/// `header_size` is `custom_header_length + NUM_FACET_SIZE` (the offset at which
/// the facet records begin), matching admesh `stl_open_count_facets`
/// (stlinit.cpp:66-94).
fn read_stl_binary(data: &[u8], header_size: usize) -> Result<TriangleMesh> {
    // admesh/stlinit.cpp:89 — reject files whose size doesn't line up with a
    // whole number of facets, or that are below STL_MIN_FILE_SIZE.
    let file_size = data.len();
    if file_size < header_size
        || (file_size - header_size) % SIZEOF_STL_FACET != 0
        || file_size < STL_MIN_FILE_SIZE
    {
        return Err(Error::Mesh(
            "stl_open_count_facets: The file has the wrong size.".into(),
        ));
    }
    // admesh/stlinit.cpp:94 — `num_facets = (file_size - header_size) / SIZEOF_STL_FACET;`
    let num_facets = (file_size - header_size) / SIZEOF_STL_FACET;

    // admesh/stlinit.cpp:100-108 — the uint32 following the header should hold
    // the facet count; admesh only logs a warning on mismatch, it does not fail.

    let body = &data[header_size..];
    if body.len() < num_facets * SIZEOF_STL_FACET {
        return Err(Error::Mesh("Binary STL truncated".into()));
    }

    // We store unique vertices via a simple hash-dedup approach.
    let mut vertices: Vec<Point3F> = Vec::new();
    let mut indices: Vec<Triangle> = Vec::with_capacity(num_facets);
    let mut vertex_map: HashMap<[u32; 3], u32> = HashMap::new();

    for i in 0..num_facets {
        let offset = i * SIZEOF_STL_FACET;
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
    custom_header_length: i32,
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
    let mesh = match read_stl_file_with_header_length(path, custom_header_length) {
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

    /// A unit cube: 8 vertices, 12 facets. As a binary STL this is
    /// `80 + 4 + 12*50 = 684` bytes, comfortably above admesh's
    /// `STL_MIN_FILE_SIZE` (284), so the faithful binary/ASCII detection
    /// (stlinit.cpp:74-94) classifies and parses it as binary.
    fn cube_mesh() -> TriangleMesh {
        let verts = vec![
            Point3F::new(0.0, 0.0, 0.0),
            Point3F::new(1.0, 0.0, 0.0),
            Point3F::new(1.0, 1.0, 0.0),
            Point3F::new(0.0, 1.0, 0.0),
            Point3F::new(0.0, 0.0, 1.0),
            Point3F::new(1.0, 0.0, 1.0),
            Point3F::new(1.0, 1.0, 1.0),
            Point3F::new(0.0, 1.0, 1.0),
        ];
        let tris = vec![
            Triangle::new(0, 1, 2),
            Triangle::new(0, 2, 3),
            Triangle::new(4, 6, 5),
            Triangle::new(4, 7, 6),
            Triangle::new(0, 4, 5),
            Triangle::new(0, 5, 1),
            Triangle::new(1, 5, 6),
            Triangle::new(1, 6, 2),
            Triangle::new(2, 6, 7),
            Triangle::new(2, 7, 3),
            Triangle::new(3, 7, 4),
            Triangle::new(3, 4, 0),
        ];
        TriangleMesh::from_parts(verts, tris)
    }

    #[test]
    fn test_roundtrip_binary_stl() {
        let mesh = cube_mesh();

        let dir = std::env::temp_dir().join("test_stl_roundtrip");
        std::fs::create_dir_all(&dir).unwrap();
        let file_path = dir.join("test.stl");

        assert!(store_stl(&file_path, &mesh, true));
        let loaded = read_stl_file(&file_path).unwrap();
        assert_eq!(loaded.vertex_count(), 8);
        assert_eq!(loaded.indices().len(), 12);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn test_roundtrip_ascii_stl() {
        // admesh `stl_open_count_facets` requires a full 128-byte window past
        // the header before it will classify a file (stlinit.cpp:69), so use a
        // mesh large enough to exceed that threshold for both encodings.
        let mesh = cube_mesh();

        let dir = std::env::temp_dir().join("test_stl_ascii_roundtrip");
        std::fs::create_dir_all(&dir).unwrap();
        let file_path = dir.join("test_ascii.stl");

        assert!(store_stl(&file_path, &mesh, false));
        let loaded = read_stl_file(&file_path).unwrap();
        assert_eq!(loaded.vertex_count(), 8);
        assert_eq!(loaded.indices().len(), 12);

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn test_load_stl_into_model() {
        // STL.cpp:17-40 — fills a Model with one object named after the file.
        let mesh = cube_mesh();

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
