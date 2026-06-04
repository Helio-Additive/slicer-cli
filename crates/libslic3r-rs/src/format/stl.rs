//! STL file loading and storing.
//!
//! C++ Reference:
//! - Format/STL.hpp
//! - Format/STL.cpp
//!
//! Supports both binary and ASCII STL formats.

use crate::geometry::Point3F;
use crate::model::{Model, ModelObject};
use crate::triangle_mesh::{Triangle, TriangleMesh};
use crate::{Error, Result};

use std::io::Write;
use std::path::Path;

#[cfg(target_os = "windows")]
const DIR_SEPARATOR: char = '\\';
#[cfg(not(target_os = "windows"))]
const DIR_SEPARATOR: char = '/';

/// Optional progress callback for STL import.
/// `(current_bytes, total_bytes)` -- return `false` to cancel.
pub type ImportStlProgressFn = Box<dyn Fn(usize, usize) -> bool>;

// ---------------------------------------------------------------------------
// Binary STL constants
// ---------------------------------------------------------------------------

const STL_HEADER_SIZE: usize = 80;
const STL_FACET_SIZE: usize = 50; // 12 floats (normal + 3 vertices) + 2 bytes attribute

// ---------------------------------------------------------------------------
// Loading  (STL.cpp:17-40)
// ---------------------------------------------------------------------------

/// Load an STL file (binary or ASCII) into a `TriangleMesh`.
fn read_stl_file(path: &Path) -> Result<TriangleMesh> {
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
    let mut vertex_map: std::collections::HashMap<[u32; 3], u32> = std::collections::HashMap::new();

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
    let mut vertex_map: std::collections::HashMap<[u32; 3], u32> = std::collections::HashMap::new();
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

/// Load an STL file into a `Model`.
/// STL.cpp:17-40
pub fn load_stl(
    path: &Path,
    object_name: Option<&str>,
    _progress_fn: Option<ImportStlProgressFn>,
) -> Result<Model> {
    let mesh = read_stl_file(path)?;

    if mesh.is_empty() {
        return Err(Error::Mesh(
            "This STL file couldn't be read because it's empty.".into(),
        ));
    }

    let name = match object_name {
        Some(n) => n.to_string(),
        None => {
            let path_str = path.to_string_lossy();
            match path_str.rfind(DIR_SEPARATOR) {
                Some(pos) => path_str[pos + 1..].to_string(),
                None => path_str.to_string(),
            }
        }
    };

    let obj = ModelObject::new(name, mesh);
    let mut model = Model::new();
    model.add_object(obj);
    Ok(model)
}

/// Store a `TriangleMesh` as a binary STL file.
/// STL.cpp:42-50
pub fn store_stl(path: &Path, mesh: &TriangleMesh, binary: bool) -> Result<()> {
    if binary {
        write_binary_stl(path, mesh)
    } else {
        write_ascii_stl(path, mesh)
    }
}

/// Store a `Model` as an STL (merges all objects).
/// STL.cpp:57-62
pub fn store_stl_model(path: &Path, model: &Model, binary: bool) -> Result<()> {
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
    let merged = TriangleMesh::from_parts(all_verts, all_tris);
    store_stl(path, &merged, binary)
}

// ---------------------------------------------------------------------------
// Writers
// ---------------------------------------------------------------------------

fn write_binary_stl(path: &Path, mesh: &TriangleMesh) -> Result<()> {
    let mut file = std::fs::File::create(path)
        .map_err(|e| Error::IO(format!("Failed to create STL file: {}", e)))?;

    // 80-byte header
    let header = [0u8; STL_HEADER_SIZE];
    file.write_all(&header)
        .map_err(|e| Error::IO(format!("Write error: {}", e)))?;

    // Facet count
    let num_facets = mesh.indices().len() as u32;
    file.write_all(&num_facets.to_le_bytes())
        .map_err(|e| Error::IO(format!("Write error: {}", e)))?;

    let verts = mesh.vertices();
    for tri in mesh.indices() {
        let v0 = &verts[tri.indices[0] as usize];
        let v1 = &verts[tri.indices[1] as usize];
        let v2 = &verts[tri.indices[2] as usize];

        // Compute face normal
        let ux = v1.x() - v0.x();
        let uy = v1.y() - v0.y();
        let uz = v1.z() - v0.z();
        let vx = v2.x() - v0.x();
        let vy = v2.y() - v0.y();
        let vz = v2.z() - v0.z();
        let nx = (uy * vz - uz * vy) as f32;
        let ny = (uz * vx - ux * vz) as f32;
        let nz = (ux * vy - uy * vx) as f32;
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        let (nx, ny, nz) = if len > 0.0 {
            (nx / len, ny / len, nz / len)
        } else {
            (0.0, 0.0, 0.0)
        };

        file.write_all(&nx.to_le_bytes())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
        file.write_all(&ny.to_le_bytes())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
        file.write_all(&nz.to_le_bytes())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;

        for vi in &tri.indices {
            let v = &verts[*vi as usize];
            file.write_all(&(v.x() as f32).to_le_bytes())
                .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
            file.write_all(&(v.y() as f32).to_le_bytes())
                .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
            file.write_all(&(v.z() as f32).to_le_bytes())
                .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
        }

        // Attribute byte count
        file.write_all(&0u16.to_le_bytes())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }
    Ok(())
}

fn write_ascii_stl(path: &Path, mesh: &TriangleMesh) -> Result<()> {
    let mut file = std::fs::File::create(path)
        .map_err(|e| Error::IO(format!("Failed to create STL file: {}", e)))?;

    writeln!(file, "solid mesh").map_err(|e| Error::IO(format!("Write error: {}", e)))?;

    let verts = mesh.vertices();
    for tri in mesh.indices() {
        let v0 = &verts[tri.indices[0] as usize];
        let v1 = &verts[tri.indices[1] as usize];
        let v2 = &verts[tri.indices[2] as usize];

        let ux = v1.x() - v0.x();
        let uy = v1.y() - v0.y();
        let uz = v1.z() - v0.z();
        let vx = v2.x() - v0.x();
        let vy = v2.y() - v0.y();
        let vz = v2.z() - v0.z();
        let nx = (uy * vz - uz * vy) as f32;
        let ny = (uz * vx - ux * vz) as f32;
        let nz = (ux * vy - uy * vx) as f32;
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        let (nx, ny, nz) = if len > 0.0 {
            (nx / len, ny / len, nz / len)
        } else {
            (0.0, 0.0, 0.0)
        };

        writeln!(file, "  facet normal {} {} {}", nx, ny, nz)
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
        writeln!(file, "    outer loop").map_err(|e| Error::IO(format!("Write error: {}", e)))?;
        for vi in &tri.indices {
            let v = &verts[*vi as usize];
            writeln!(
                file,
                "      vertex {} {} {}",
                v.x() as f32,
                v.y() as f32,
                v.z() as f32
            )
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
        }
        writeln!(file, "    endloop").map_err(|e| Error::IO(format!("Write error: {}", e)))?;
        writeln!(file, "  endfacet").map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }

    writeln!(file, "endsolid mesh").map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    Ok(())
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

        store_stl(&file_path, &mesh, true).unwrap();
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

        store_stl(&file_path, &mesh, false).unwrap();
        let loaded = read_stl_file(&file_path).unwrap();
        assert_eq!(loaded.vertex_count(), 3);
        assert_eq!(loaded.indices().len(), 1);

        std::fs::remove_dir_all(&dir).ok();
    }
}
