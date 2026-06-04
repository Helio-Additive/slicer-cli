//! OBJ file format support.
//!
//! This module provides loading and saving of OBJ files,
//! mirroring BambuStudio's Format/OBJ.cpp.
//!
//! OBJ is a simple text-based format that includes:
//! - Vertex positions (v)
//! - Texture coordinates (vt)
//! - Normals (vn)
//! - Faces (f)
//!
//! # Example OBJ File
//!
//! ```
//! # Comment
//! v 0.0 0.0 0.0
//! v 1.0 0.0 0.0
//! v 1.0 1.0 0.0
//! v 0.0 1.0 0.0
//! f 1 2 3
//! f 1 3 4
//! ```
//! - `src/libslic3r/Format/OBJ.cpp`

use crate::geometry::Point3F;
use crate::model::{Model, ModelObject};
use crate::triangle_mesh::TriangleMesh;
use crate::{Error, Result};
use std::io::{Read, Write};
use std::path::Path;

/// Load an OBJ file into a Model, combining mesh loading and model construction
/// Format/OBJ.cpp:247-264
pub fn load_obj<P: AsRef<Path>>(path: P) -> Result<Model> {
    // Format/OBJ.cpp:249
    let mut file = std::fs::File::open(path.as_ref())
        .map_err(|e| Error::Mesh(format!("Failed to open OBJ file: {}", e)))?;

    // Format/OBJ.cpp:32
    let mut contents = String::new();
    // Format/OBJ.cpp:32
    file.read_to_string(&mut contents)
        .map_err(|e| Error::Mesh(format!("Failed to read OBJ file: {}", e)))?;

    // Format/OBJ.cpp:253-261
    parse_obj(&contents)
}

/// Load meshes from OBJ file (legacy API), extracts mesh from each model object
/// Format/OBJ.cpp:247-264
pub fn load_obj_meshes<P: AsRef<Path>>(path: P) -> Result<Vec<TriangleMesh>> {
    // Format/OBJ.cpp:251
    let model = load_obj(path)?;
    // Format/OBJ.cpp:253
    Ok(model.objects.into_iter().map(|o| o.mesh).collect())
}

/// Parse OBJ text content into a Model with vertices and faces
/// Format/OBJ.cpp:25-244
fn parse_obj(content: &str) -> Result<Model> {
    // Format/OBJ.cpp:27-28
    let mut model = Model::new();
    // Format/OBJ.cpp:30
    let mut vertices: Vec<Point3F> = Vec::new();
    // Format/OBJ.cpp:30
    let mut triangles: Vec<[u32; 3]> = Vec::new();

    // Format/OBJ.cpp:32
    for line in content.lines() {
        // Format/OBJ.cpp:32
        let trimmed = line.trim();

        // Format/OBJ.cpp:32
        if trimmed.is_empty() || trimmed.starts_with('#') {
            // Format/OBJ.cpp:32
            continue;
        }

        // Format/OBJ.cpp:32
        let parts: Vec<&str> = trimmed.split_whitespace().collect();
        // Format/OBJ.cpp:32
        if parts.is_empty() {
            // Format/OBJ.cpp:32
            continue;
        }

        // Format/OBJ.cpp:82-234
        match parts[0] {
            "v" => {
                // Format/OBJ.cpp:115-128
                if parts.len() >= 4 {
                    // Format/OBJ.cpp:116-117
                    if let (Ok(x), Ok(y), Ok(z)) = (
                        parts[1].parse::<f64>(),
                        parts[2].parse::<f64>(),
                        parts[3].parse::<f64>(),
                    ) {
                        // Format/OBJ.cpp:117
                        vertices.push(Point3F::new(x, y, z));
                    }
                }
            }
            "f" => {
                // Format/OBJ.cpp:131-234
                if parts.len() >= 4 {
                    // Format/OBJ.cpp:137-148
                    let indices: Vec<Option<u32>> = parts[1..]
                        .iter()
                        .map(|p| {
                            // Format/OBJ.cpp:137
                            let idx_str = p.split('/').next().unwrap_or(p);
                            // Format/OBJ.cpp:146
                            idx_str.parse::<u32>().ok().map(|i| i.saturating_sub(1))
                        })
                        .collect();

                    // Format/OBJ.cpp:150-153
                    if indices.len() >= 3 && indices.iter().all(|i| i.is_some()) {
                        // Format/OBJ.cpp:153
                        let first = indices[0].unwrap();

                        // Format/OBJ.cpp:153
                        for i in 1..indices.len() - 1 {
                            // Format/OBJ.cpp:153
                            let second = indices[i].unwrap();
                            // Format/OBJ.cpp:153
                            let third = indices[i + 1].unwrap();

                            // Format/OBJ.cpp:141-145
                            if (first as usize) < vertices.len()
                                && (second as usize) < vertices.len()
                                && (third as usize) < vertices.len()
                            {
                                // Format/OBJ.cpp:153
                                triangles.push([first, second, third]);
                            }
                        }
                    }
                }
            }
            "o" | "g" => {
                // Format/OBJ.cpp:38-40
                if !vertices.is_empty() && !triangles.is_empty() {
                    // Format/OBJ.cpp:236
                    let mesh = build_triangle_mesh(&vertices, &triangles);
                    // Format/OBJ.cpp:254-259
                    let name = parts.get(1).copied().unwrap_or("");
                    // Format/OBJ.cpp:260
                    let obj = ModelObject::new(name, mesh);
                    // Format/OBJ.cpp:260
                    model.objects.push(obj);

                    // Format/OBJ.cpp:30
                    vertices.clear();
                    // Format/OBJ.cpp:30
                    triangles.clear();
                }
            }
            _ => {
                // Format/OBJ.cpp:43-78
            }
        }
    }

    // Format/OBJ.cpp:236
    if !vertices.is_empty() && !triangles.is_empty() {
        // Format/OBJ.cpp:236
        let mesh = build_triangle_mesh(&vertices, &triangles);
        // Format/OBJ.cpp:260
        let obj = ModelObject::new("", mesh);
        // Format/OBJ.cpp:260
        model.objects.push(obj);
    }

    // Format/OBJ.cpp:237-241
    if model.objects.is_empty() {
        // Format/OBJ.cpp:238-240
        return Err(Error::Mesh(
            "No valid geometry found in OBJ file".to_string(),
        ));
    }

    // Format/OBJ.cpp:242-244
    Ok(model)
}

/// Build a TriangleMesh from parsed OBJ vertex and triangle index data
/// Format/OBJ.cpp:105-108
fn build_triangle_mesh(vertices: &[Point3F], triangles: &[[u32; 3]]) -> TriangleMesh {
    // Format/OBJ.cpp:107-108
    let mut mesh = TriangleMesh::with_capacity(vertices.len(), triangles.len());

    // Format/OBJ.cpp:115-117
    for vertex in vertices {
        // Format/OBJ.cpp:117
        mesh.add_vertex(*vertex);
    }

    // Format/OBJ.cpp:153
    for tri_indices in triangles {
        // Format/OBJ.cpp:153
        mesh.add_triangle_indices(tri_indices[0], tri_indices[1], tri_indices[2]);
    }

    // Format/OBJ.cpp:236
    mesh
}

/// Save a Model to an OBJ file by writing vertex and face data
/// Format/OBJ.cpp:266-271
pub fn save_obj<P: AsRef<Path>>(path: P, model: &Model) -> Result<()> {
    // Format/OBJ.cpp:266
    let mut file = std::fs::File::create(path.as_ref())
        .map_err(|e| Error::Mesh(format!("Failed to create OBJ file: {}", e)))?;

    // Format/OBJ.cpp:269
    let obj_content = generate_obj(model)?;

    // Format/OBJ.cpp:269
    file.write_all(obj_content.as_bytes())
        .map_err(|e| Error::Mesh(format!("Failed to write OBJ file: {}", e)))?;

    // Format/OBJ.cpp:270
    Ok(())
}

/// Generate OBJ text content from a Model (handles multiple objects)
/// Format/OBJ.cpp:279-283
fn generate_obj(model: &Model) -> Result<String> {
    // Format/OBJ.cpp:269
    let mut obj = String::new();

    // Format/OBJ.cpp:269
    obj.push_str("# Generated by Rust Slicer\n");
    // Format/OBJ.cpp:269
    obj.push_str("# OBJ file format\n\n");

    // Format/OBJ.cpp:279-283
    let mut vertex_offset: u32 = 0;

    // Format/OBJ.cpp:273-277
    for (obj_idx, object) in model.objects.iter().enumerate() {
        // Format/OBJ.cpp:254-259
        obj.push_str(&format!("# Object {}\n", obj_idx));
        // Format/OBJ.cpp:254-259
        if !object.name.is_empty() {
            // Format/OBJ.cpp:259
            obj.push_str(&format!("o {}\n", object.name));
        } else {
            // Format/OBJ.cpp:257
            obj.push_str(&format!("o object{}\n", obj_idx));
        }

        // Format/OBJ.cpp:115-117
        let vertices = object.mesh.vertices();
        // Format/OBJ.cpp:115-117
        for vertex in vertices {
            // Format/OBJ.cpp:117
            obj.push_str(&format!("v {} {} {}\n", vertex.x, vertex.y, vertex.z));
        }

        // Format/OBJ.cpp:153
        let indices = object.mesh.indices();
        // Format/OBJ.cpp:153
        for tri in indices {
            // Format/OBJ.cpp:153
            let v1 = tri.indices[0] + 1 + vertex_offset;
            // Format/OBJ.cpp:153
            let v2 = tri.indices[1] + 1 + vertex_offset;
            // Format/OBJ.cpp:153
            let v3 = tri.indices[2] + 1 + vertex_offset;
            // Format/OBJ.cpp:153
            obj.push_str(&format!("f {} {} {}\n", v1, v2, v3));
        }

        // Format/OBJ.cpp:279-283
        vertex_offset += vertices.len() as u32;
        // Format/OBJ.cpp:279-283
        obj.push('\n');
    }

    // Format/OBJ.cpp:283
    Ok(obj)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_obj_vertices() {
        let content = "v 0.0 0.0 0.0\nv 1.0 0.0 0.0\nv 1.0 1.0 0.0";
        let result = parse_obj(content);
        assert!(result.is_ok());
    }

    #[test]
    fn test_parse_obj_empty() {
        let result = parse_obj("");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_obj_comments() {
        let content = "# This is a comment\nv 0.0 0.0 0.0";
        let result = parse_obj(content);
        assert!(result.is_ok());
    }

    #[test]
    fn test_generate_obj_empty() {
        let model = Model::new();
        let result = generate_obj(&model);
        assert!(result.is_ok());
    }
}
