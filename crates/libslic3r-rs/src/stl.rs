//! STL file loading and saving.
//!
//! This module provides functions to load and save STL files,
//! supporting both ASCII and binary formats.
//!
//! C++ Reference: Format/STL.cpp

use super::{Triangle, TriangleMesh};
use crate::geometry::Point3F;
use crate::{Error, Result};
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Write};
use std::path::Path;

/// Load a triangle mesh from an STL file, auto-detecting ASCII vs binary format.
/// Format/STL.cpp:17-40
pub fn load_stl<P: AsRef<Path>>(path: P) -> Result<TriangleMesh> {
    // Format/STL.cpp:19
    let path = path.as_ref();
    // Format/STL.cpp:17
    let file = File::open(path).map_err(|e| Error::Io(e))?;
    // Format/STL.cpp:22
    let mut reader = BufReader::new(file);

    // Read the first 80 bytes to check format
    // Format/STL.cpp:22
    let mut header = [0u8; 80];
    // Format/STL.cpp:22
    reader.read_exact(&mut header).map_err(|e| Error::Io(e))?;

    // Check if it's ASCII by looking for "solid" at the start
    // Note: Some binary files also start with "solid", so we need additional checks
    // Format/STL.cpp:22
    let header_str = String::from_utf8_lossy(&header);
    // Format/STL.cpp:22
    let is_ascii = header_str.trim_start().starts_with("solid") && is_likely_ascii(&header);

    // Reset to beginning by re-opening file
    // Format/STL.cpp:22
    drop(reader);
    // Format/STL.cpp:17
    let file = File::open(path).map_err(|e| Error::Io(e))?;

    // Format/STL.cpp:22-25
    if is_ascii {
        load_stl_ascii(BufReader::new(file))
    } else {
        load_stl_binary(BufReader::new(file))
    }
}

/// Check if the header suggests ASCII format by looking for null/non-printable bytes.
/// Format/STL.cpp:22
fn is_likely_ascii(header: &[u8]) -> bool {
    // If there are null bytes or many non-printable characters, it's likely binary
    // Format/STL.cpp:22
    let non_printable_count = header
        .iter()
        .filter(|&&b| b == 0 || (b < 32 && b != b'\n' && b != b'\r' && b != b'\t'))
        .count();
    // Format/STL.cpp:22
    non_printable_count == 0
}

/// Load an ASCII STL file by parsing vertex and facet lines.
/// Format/STL.cpp:22
fn load_stl_ascii<R: BufRead>(reader: R) -> Result<TriangleMesh> {
    // Format/STL.cpp:19
    let mut mesh = TriangleMesh::new();
    // Format/STL.cpp:22
    let mut vertices: Vec<Point3F> = Vec::new();

    // Format/STL.cpp:22
    for line in reader.lines() {
        // Format/STL.cpp:22
        let line = line.map_err(|e| Error::Io(e))?;
        // Format/STL.cpp:22
        let line = line.trim();

        // Format/STL.cpp:22
        if line.starts_with("vertex") {
            // Format/STL.cpp:22
            let parts: Vec<&str> = line.split_whitespace().collect();
            // Format/STL.cpp:22
            if parts.len() >= 4 {
                // Format/STL.cpp:22
                let x: f64 = parts[1]
                    .parse()
                    .map_err(|_| Error::Mesh("Invalid vertex X coordinate".into()))?;
                // Format/STL.cpp:22
                let y: f64 = parts[2]
                    .parse()
                    .map_err(|_| Error::Mesh("Invalid vertex Y coordinate".into()))?;
                // Format/STL.cpp:22
                let z: f64 = parts[3]
                    .parse()
                    .map_err(|_| Error::Mesh("Invalid vertex Z coordinate".into()))?;
                // Format/STL.cpp:22
                vertices.push(Point3F::new(x, y, z));
            }
        } else {
            // Format/STL.cpp:22
            if line.starts_with("endfacet") {
                // End of a facet - we should have 3 vertices
                // Format/STL.cpp:22
                if vertices.len() >= 3 {
                    // Format/STL.cpp:22
                    let base_idx = mesh.vertex_count() as u32;
                    // Format/STL.cpp:22
                    for v in vertices.drain(..) {
                        // Format/STL.cpp:22
                        mesh.add_vertex(v);
                    }
                    // Format/STL.cpp:22
                    mesh.add_triangle(Triangle::new(base_idx, base_idx + 1, base_idx + 2));
                }
                // Format/STL.cpp:22
                vertices.clear();
            }
        }
    }

    // Check mesh is not empty after loading
    // Format/STL.cpp:26-28
    if mesh.is_empty() {
        // Format/STL.cpp:26-28
        return Err(Error::Mesh("No triangles found in STL file".into()));
    }

    // Format/STL.cpp:38
    Ok(mesh)
}

/// Load a binary STL file by reading the 80-byte header, triangle count, and triangle data.
/// Format/STL.cpp:22
fn load_stl_binary<R: Read>(mut reader: R) -> Result<TriangleMesh> {
    // Skip 80-byte header
    // Format/STL.cpp:22
    let mut header = [0u8; 80];
    // Format/STL.cpp:22
    reader.read_exact(&mut header).map_err(|e| Error::Io(e))?;

    // Read triangle count (4 bytes, little-endian)
    // Format/STL.cpp:22
    let mut count_bytes = [0u8; 4];
    // Format/STL.cpp:22
    reader
        .read_exact(&mut count_bytes)
        .map_err(|e| Error::Io(e))?;
    // Format/STL.cpp:22
    let triangle_count = u32::from_le_bytes(count_bytes) as usize;

    // Format/STL.cpp:19
    let mut mesh = TriangleMesh::with_capacity(triangle_count * 3, triangle_count);

    // Each triangle is 50 bytes:
    // - Normal: 3 floats (12 bytes)
    // - Vertex 1-3: 3 floats each (12 bytes each)
    // - Attribute byte count: 2 bytes
    // Format/STL.cpp:22
    let mut triangle_data = [0u8; 50];

    // Format/STL.cpp:22
    for _ in 0..triangle_count {
        // Format/STL.cpp:22
        reader
            .read_exact(&mut triangle_data)
            .map_err(|e| Error::Io(e))?;

        // Skip normal (bytes 0-11), read vertices
        // Format/STL.cpp:22
        let v1 = read_vertex(&triangle_data[12..24]);
        // Format/STL.cpp:22
        let v2 = read_vertex(&triangle_data[24..36]);
        // Format/STL.cpp:22
        let v3 = read_vertex(&triangle_data[36..48]);

        // Format/STL.cpp:22
        let base_idx = mesh.vertex_count() as u32;
        // Format/STL.cpp:22
        mesh.add_vertex(v1);
        // Format/STL.cpp:22
        mesh.add_vertex(v2);
        // Format/STL.cpp:22
        mesh.add_vertex(v3);
        // Format/STL.cpp:22
        mesh.add_triangle(Triangle::new(base_idx, base_idx + 1, base_idx + 2));
    }

    // Check mesh is not empty after loading
    // Format/STL.cpp:26-28
    if mesh.is_empty() {
        // Format/STL.cpp:26-28
        return Err(Error::Mesh("No triangles found in STL file".into()));
    }

    // Format/STL.cpp:39
    Ok(mesh)
}

/// Read a vertex (3 floats) from bytes in little-endian format.
/// Format/STL.cpp:22
fn read_vertex(data: &[u8]) -> Point3F {
    // Format/STL.cpp:22
    let x = f32::from_le_bytes([data[0], data[1], data[2], data[3]]) as f64;
    // Format/STL.cpp:22
    let y = f32::from_le_bytes([data[4], data[5], data[6], data[7]]) as f64;
    // Format/STL.cpp:22
    let z = f32::from_le_bytes([data[8], data[9], data[10], data[11]]) as f64;
    // Format/STL.cpp:22
    Point3F::new(x, y, z)
}

/// Save a triangle mesh to an STL file (binary format by default).
/// Format/STL.cpp:42-50
pub fn save_stl<P: AsRef<Path>>(path: P, mesh: &TriangleMesh) -> Result<()> {
    // Format/STL.cpp:44
    save_stl_binary(path, mesh)
}

/// Save a triangle mesh to a binary STL file.
/// Format/STL.cpp:42-50
pub fn save_stl_binary<P: AsRef<Path>>(path: P, mesh: &TriangleMesh) -> Result<()> {
    // Format/STL.cpp:45
    let file = File::create(path).map_err(|e| Error::Io(e))?;
    // Format/STL.cpp:45
    let mut writer = BufWriter::new(file);

    // Write 80-byte header (must be exactly 80 bytes)
    // Format/STL.cpp:45
    let header = b"Binary STL generated by Slicer\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";
    // Format/STL.cpp:45
    debug_assert_eq!(header.len(), 80, "STL header must be exactly 80 bytes");
    // Format/STL.cpp:45
    writer.write_all(header).map_err(|e| Error::Io(e))?;

    // Write triangle count
    // Format/STL.cpp:45
    let count = mesh.triangle_count() as u32;
    // Format/STL.cpp:45
    writer
        .write_all(&count.to_le_bytes())
        .map_err(|e| Error::Io(e))?;

    // Write each triangle
    // Format/STL.cpp:45
    for i in 0..mesh.triangle_count() {
        // Format/STL.cpp:45
        let [v0, v1, v2] = mesh.triangle_vertices(i);

        // Calculate normal
        // Format/STL.cpp:45
        let e1 = v1 - v0;
        // Format/STL.cpp:45
        let e2 = v2 - v0;
        // Format/STL.cpp:45
        let normal = e1.cross(&e2).normalize();

        // Write normal
        // Format/STL.cpp:45
        write_float(&mut writer, normal.x as f32)?;
        // Format/STL.cpp:45
        write_float(&mut writer, normal.y as f32)?;
        // Format/STL.cpp:45
        write_float(&mut writer, normal.z as f32)?;

        // Write vertices
        // Format/STL.cpp:45
        write_float(&mut writer, v0.x as f32)?;
        // Format/STL.cpp:45
        write_float(&mut writer, v0.y as f32)?;
        // Format/STL.cpp:45
        write_float(&mut writer, v0.z as f32)?;

        // Format/STL.cpp:45
        write_float(&mut writer, v1.x as f32)?;
        // Format/STL.cpp:45
        write_float(&mut writer, v1.y as f32)?;
        // Format/STL.cpp:45
        write_float(&mut writer, v1.z as f32)?;

        // Format/STL.cpp:45
        write_float(&mut writer, v2.x as f32)?;
        // Format/STL.cpp:45
        write_float(&mut writer, v2.y as f32)?;
        // Format/STL.cpp:45
        write_float(&mut writer, v2.z as f32)?;

        // Write attribute byte count (0)
        // Format/STL.cpp:45
        writer.write_all(&[0u8, 0u8]).map_err(|e| Error::Io(e))?;
    }

    // Format/STL.cpp:49
    writer.flush().map_err(|e| Error::Io(e))?;
    // Format/STL.cpp:49
    Ok(())
}

/// Write a float in little-endian format.
/// Format/STL.cpp:45
fn write_float<W: Write>(writer: &mut W, value: f32) -> Result<()> {
    // Format/STL.cpp:45
    writer
        .write_all(&value.to_le_bytes())
        .map_err(|e| Error::Io(e))
}

/// Save a triangle mesh to an ASCII STL file.
/// Format/STL.cpp:42-50
pub fn save_stl_ascii<P: AsRef<Path>>(path: P, mesh: &TriangleMesh) -> Result<()> {
    // Format/STL.cpp:47
    let file = File::create(path).map_err(|e| Error::Io(e))?;
    // Format/STL.cpp:47
    let mut writer = BufWriter::new(file);

    // Format/STL.cpp:47
    writeln!(writer, "solid mesh").map_err(|e| Error::Io(e))?;

    // Format/STL.cpp:47
    for i in 0..mesh.triangle_count() {
        // Format/STL.cpp:47
        let [v0, v1, v2] = mesh.triangle_vertices(i);

        // Calculate normal
        // Format/STL.cpp:47
        let e1 = v1 - v0;
        // Format/STL.cpp:47
        let e2 = v2 - v0;
        // Format/STL.cpp:47
        let normal = e1.cross(&e2).normalize();

        // Format/STL.cpp:47
        writeln!(
            writer,
            "  facet normal {} {} {}",
            normal.x, normal.y, normal.z
        )
        .map_err(|e| Error::Io(e))?;
        // Format/STL.cpp:47
        writeln!(writer, "    outer loop").map_err(|e| Error::Io(e))?;
        // Format/STL.cpp:47
        writeln!(writer, "      vertex {} {} {}", v0.x, v0.y, v0.z).map_err(|e| Error::Io(e))?;
        // Format/STL.cpp:47
        writeln!(writer, "      vertex {} {} {}", v1.x, v1.y, v1.z).map_err(|e| Error::Io(e))?;
        // Format/STL.cpp:47
        writeln!(writer, "      vertex {} {} {}", v2.x, v2.y, v2.z).map_err(|e| Error::Io(e))?;
        // Format/STL.cpp:47
        writeln!(writer, "    endloop").map_err(|e| Error::Io(e))?;
        // Format/STL.cpp:47
        writeln!(writer, "  endfacet").map_err(|e| Error::Io(e))?;
    }

    // Format/STL.cpp:47
    writeln!(writer, "endsolid mesh").map_err(|e| Error::Io(e))?;
    // Format/STL.cpp:47
    writer.flush().map_err(|e| Error::Io(e))?;
    // Format/STL.cpp:49
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn test_load_ascii_stl() {
        let stl_content = r#"solid test
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
  facet normal 0 0 1
    outer loop
      vertex 1 0 0
      vertex 1 1 0
      vertex 0 1 0
    endloop
  endfacet
endsolid test"#;

        let reader = BufReader::new(Cursor::new(stl_content));
        let mesh = load_stl_ascii(reader).unwrap();

        assert_eq!(mesh.triangle_count(), 2);
        assert_eq!(mesh.vertex_count(), 6);
    }

    #[test]
    fn test_read_vertex() {
        // Test reading a vertex from bytes
        let x: f32 = 1.5;
        let y: f32 = 2.5;
        let z: f32 = 3.5;

        let mut data = Vec::new();
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());

        let v = read_vertex(&data);
        assert!((v.x - 1.5).abs() < 1e-6);
        assert!((v.y - 2.5).abs() < 1e-6);
        assert!((v.z - 3.5).abs() < 1e-6);
    }

    #[test]
    fn test_roundtrip_binary() {
        use tempfile::tempdir;

        let dir = tempdir().unwrap();
        let path = dir.path().join("test.stl");

        // Create a simple mesh
        let mesh = TriangleMesh::cube(10.0);

        // Save it
        save_stl_binary(&path, &mesh).unwrap();

        // Load it back
        let loaded = load_stl(&path).unwrap();

        assert_eq!(loaded.triangle_count(), mesh.triangle_count());
    }

    #[test]
    fn test_roundtrip_ascii() {
        use tempfile::tempdir;

        let dir = tempdir().unwrap();
        let path = dir.path().join("test.stl");

        // Create a simple mesh
        let mesh = TriangleMesh::cube(10.0);

        // Save it as ASCII
        save_stl_ascii(&path, &mesh).unwrap();

        // Load it back
        let loaded = load_stl(&path).unwrap();

        assert_eq!(loaded.triangle_count(), mesh.triangle_count());
    }
}
