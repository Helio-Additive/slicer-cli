//! 3MF (3D Manufacturing Format) file support.
//!
//! This module provides loading and saving of 3MF files according to the
//! 3MF specification (ISO/ASTM 52915:2020).
//!
//! 3MF is a ZIP-based container format with XML content that includes:
//! - 3D model data in XML format
//! - Thumbnail images
//! - Print tickets
//! - Relationships between parts
//!
//! # 3MF File Structure
//!
//! ```
//! model.3mf (ZIP archive)
//! ├── [Content_Types].xml
//! ├── _rels/.rels
//! ├── 3D/3dmodel.model
//! ├── 3D/_rels/3dmodel.model.rels
//! ├── 3D/Textures/ (optional)
//! └── Metadata/thumbnail.png (optional)
//! ```
//!
//! # BambuStudio Reference
//!
//! This implementation follows:
//! - `src/libslic3r/Format/3mf.cpp`
//! - 3MF Core Specification 2.0
//! - 3MF Materials Extension
//! - 3MF Production Extension

/// Import geometry and model types
/// Format/3mf.cpp:1-14
use crate::geometry::{Point3F, Transform3D};
/// Import model data structures
/// Format/3mf.cpp:3
use crate::model::{Instance, Model, ModelObject};
/// Import triangle mesh type
/// Format/3mf.cpp:3
use crate::triangle_mesh::TriangleMesh;
/// Import error handling types
/// Format/3mf.cpp:2
use crate::{Error, Result};
/// Import HashMap for object tracking
/// Format/3mf.cpp:16
use std::collections::HashMap;
/// Import I/O traits for reading and writing
/// Format/3mf.cpp:16
use std::io::{Read, Write};
/// Import Path for file system operations
/// Format/3mf.cpp:22
use std::path::Path;

/// Load a 3MF file and return a Model with all objects.
/// Format/3mf.cpp:677-702
pub fn load_3mf<P: AsRef<Path>>(path: P) -> Result<Model> {
    // Format/3mf.cpp:722-726
    let file = std::fs::File::open(path.as_ref())
        .map_err(|e| Error::Mesh(format!("Failed to open 3MF file: {}", e)))?;

    // Format/3mf.cpp:727-730
    let mut archive = zip::ZipArchive::new(file)
        .map_err(|e| Error::Mesh(format!("Invalid 3MF ZIP archive: {}", e)))?;

    // Format/3mf.cpp:738-762
    let content_types = read_content_types(&mut archive)?;

    // Format/3mf.cpp:744
    let model_entry = find_3d_model(&content_types)?;

    // Format/3mf.cpp:748
    let model_xml = read_zip_entry(&mut archive, &model_entry)?;

    // Format/3mf.cpp:955-1017
    let model = parse_3d_model(&model_xml)?;

    // Format/3mf.cpp:952
    Ok(model)
}

/// Load meshes from a 3MF file (legacy convenience API).
/// Format/3mf.cpp:3263-3277
pub fn load_3mf_meshes<P: AsRef<Path>>(path: P) -> Result<Vec<TriangleMesh>> {
    // Format/3mf.cpp:3257
    let model = load_3mf(path)?;
    // Format/3mf.cpp:3260
    Ok(model.objects.into_iter().map(|o| o.mesh).collect())
}

/// Save a Model to a 3MF file.
/// Format/3mf.cpp:2328-2445
pub fn save_3mf<P: AsRef<Path>>(path: P, model: &Model) -> Result<()> {
    // Format/3mf.cpp:2330-2336
    let file = std::fs::File::create(path.as_ref())
        .map_err(|e| Error::Mesh(format!("Failed to create 3MF file: {}", e)))?;

    // Format/3mf.cpp:2330
    let mut archive = zip::ZipWriter::new(file);

    // Format/3mf.cpp:2330
    let options = zip::write::FileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated)
        .compression_level(Some(6));

    // Format/3mf.cpp:2340-2344
    let content_types = generate_content_types(model);
    // Format/3mf.cpp:2340
    archive
        .start_file("[Content_Types].xml", options)
        .map_err(|e| Error::Mesh(format!("ZIP write error: {}", e)))?;
    // Format/3mf.cpp:2340
    archive
        .write_all(content_types.as_bytes())
        .map_err(|e| Error::Mesh(format!("Failed to write: {}", e)))?;

    // Format/3mf.cpp:2358-2362
    archive
        .start_file("_rels/.rels", options)
        .map_err(|e| Error::Mesh(format!("ZIP write error: {}", e)))?;
    // Format/3mf.cpp:2358
    archive
        .write_all(RELS_DOT_RELS.as_bytes())
        .map_err(|e| Error::Mesh(format!("Failed to write: {}", e)))?;

    // Format/3mf.cpp:2367-2371
    archive
        .start_file("3D/3dmodel.model", options)
        .map_err(|e| Error::Mesh(format!("ZIP write error: {}", e)))?;
    // Format/3mf.cpp:2514-2614
    let model_xml = generate_3d_model(model)?;
    // Format/3mf.cpp:2367
    archive
        .write_all(model_xml.as_bytes())
        .map_err(|e| Error::Mesh(format!("Failed to write: {}", e)))?;

    // Format/3mf.cpp:2484-2501
    archive
        .start_file("3D/_rels/3dmodel.model.rels", options)
        .map_err(|e| Error::Mesh(format!("ZIP write error: {}", e)))?;
    // Format/3mf.cpp:2484
    archive
        .write_all(MODEL_RELS.as_bytes())
        .map_err(|e| Error::Mesh(format!("Failed to write: {}", e)))?;

    // Format/3mf.cpp:2435-2444
    archive
        .finish()
        .map_err(|e| Error::Mesh(format!("Failed to finalize 3MF archive: {}", e)))?;

    // Format/3mf.cpp:2444
    Ok(())
}

/// Save meshes to a 3MF file (legacy convenience API).
/// Format/3mf.cpp:3263-3277
pub fn save_3mf_meshes<P: AsRef<Path>>(path: P, meshes: &[TriangleMesh]) -> Result<()> {
    // Format/3mf.cpp:3268
    let mut model = Model::new();
    // Format/3mf.cpp:2574-2576
    for (i, mesh) in meshes.iter().enumerate() {
        // Format/3mf.cpp:2578-2580
        let object = ModelObject::new(format!("Object{}", i + 1), mesh.clone());
        // Format/3mf.cpp:2580
        model.add_object(object);
    }
    // Format/3mf.cpp:3272
    save_3mf(path, &model)
}

// ============================================================================
// Content Types
// ============================================================================

#[derive(Debug)]
/// Content type entry for 3MF [Content_Types].xml
/// Format/3mf.cpp:67-79
struct ContentType {
    extension: Option<String>,
    content_type: String,
    part_name: Option<String>,
}

/// Read and parse [Content_Types].xml from the 3MF archive
/// Format/3mf.cpp:722-731
fn read_content_types<R: Read + std::io::Seek>(
    archive: &mut zip::ZipArchive<R>,
) -> Result<Vec<ContentType>> {
    // Format/3mf.cpp:722-726
    let content = read_zip_entry(archive, "[Content_Types].xml")?;
    // Format/3mf.cpp:727-731
    parse_content_types(&content)
}

/// Parse [Content_Types].xml to extract content type entries
/// Format/3mf.cpp:722-762
fn parse_content_types(xml: &str) -> Result<Vec<ContentType>> {
    // Format/3mf.cpp:724
    let mut types = Vec::new();

    // Format/3mf.cpp:738-762
    for line in xml.lines() {
        // Format/3mf.cpp:740-748
        if line.contains("<Default") {
            // Format/3mf.cpp:741
            let extension = extract_attr(line, "Extension");
            // Format/3mf.cpp:742
            let content_type = extract_attr(line, "ContentType");
            // Format/3mf.cpp:743-748
            if let (Some(ext), Some(ct)) = (extension, content_type) {
                // Format/3mf.cpp:744-747
                types.push(ContentType {
                    extension: Some(ext),
                    content_type: ct,
                    part_name: None,
                });
            }
        } else {
            // Format/3mf.cpp:749-760
            if line.contains("<Override") {
                // Format/3mf.cpp:750
                let part_name = extract_attr(line, "PartName");
                // Format/3mf.cpp:751
                let content_type = extract_attr(line, "ContentType");
                // Format/3mf.cpp:752-758
                if let (Some(pn), Some(ct)) = (part_name, content_type) {
                    // Format/3mf.cpp:753-757
                    types.push(ContentType {
                        extension: None,
                        content_type: ct,
                        part_name: Some(pn),
                    });
                }
            }
        }
    }

    // Format/3mf.cpp:762
    Ok(types)
}

/// Find the 3D model entry path from content types
/// Format/3mf.cpp:738-762
fn find_3d_model(content_types: &[ContentType]) -> Result<String> {
    // Format/3mf.cpp:744-752
    for ct in content_types {
        // Format/3mf.cpp:745
        if let Some(ref part_name) = ct.part_name {
            // Format/3mf.cpp:746-748
            if ct.content_type == "application/vnd.ms-package.3dmanufacturing-3dmodel+xml" {
                // Format/3mf.cpp:749
                return Ok(part_name.trim_start_matches('/').to_string());
            }
        }
    }

    // Format/3mf.cpp:752
    Ok("3D/3dmodel.model".to_string())
}

/// Generate [Content_Types].xml for 3MF export
/// Format/3mf.cpp:2447-2465
fn generate_content_types(_model: &Model) -> String {
    // Format/3mf.cpp:2447-2465
    r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
    <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml" />
    <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml" />
</Types>"#.to_string()
}

/// Relationships file content for 3MF root .rels
/// Format/3mf.cpp:2484-2501
const RELS_DOT_RELS: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
    <Relationship Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" Target="/3D/3dmodel.model" />
</Relationships>"#;

/// Relationships file content for 3D model .rels
/// Format/3mf.cpp:2484-2501
const MODEL_RELS: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
</Relationships>"#;

// ============================================================================
// 3D Model Parsing
// ============================================================================

/// Parse 3D model XML from 3MF archive into a Model
/// Format/3mf.cpp:955-1017
fn parse_3d_model(xml: &str) -> Result<Model> {
    // Format/3mf.cpp:957
    let mut model = Model::new();
    // Format/3mf.cpp:958
    let mut current_object: Option<ModelObjectBuilder> = None;
    // Format/3mf.cpp:959
    let mut vertices: Vec<Point3F> = Vec::new();
    // Format/3mf.cpp:960
    let mut object_transforms: HashMap<usize, Transform3D> = HashMap::new();
    // Format/3mf.cpp:961
    let mut object_builders: Vec<ModelObjectBuilder> = Vec::new();

    // Format/3mf.cpp:963
    let lines: Vec<&str> = xml.lines().collect();
    // Format/3mf.cpp:964
    let mut i = 0;

    // Format/3mf.cpp:966-1000
    while i < lines.len() {
        // Format/3mf.cpp:967
        let line = lines[i].trim();

        // Format/3mf.cpp:1681-1693
        if line.starts_with("<vertices>") {
            // Format/3mf.cpp:1682
            vertices.clear();
            // Format/3mf.cpp:1683
            i += 1;
            // Format/3mf.cpp:1684-1692
            while i < lines.len() && !lines[i].trim().starts_with("</vertices>") {
                // Format/3mf.cpp:1685
                let vertex_line = lines[i].trim();
                // Format/3mf.cpp:1694-1703
                if vertex_line.starts_with("<vertex") {
                    // Format/3mf.cpp:1698-1701
                    if let Some(v) = parse_vertex(vertex_line) {
                        // Format/3mf.cpp:1700
                        vertices.push(v);
                    }
                }
                // Format/3mf.cpp:1691
                i += 1;
            }
        } else {
            // Format/3mf.cpp:1602-1625
            if line.starts_with("<object") && !line.contains("</object>") {
                // Format/3mf.cpp:1605
                let id = extract_attr(line, "id")
                    .and_then(|s| s.parse::<usize>().ok())
                    .unwrap_or(object_builders.len());
                // Format/3mf.cpp:1608
                let name =
                    extract_attr(line, "name").unwrap_or_else(|| format!("Object{}", id + 1));
                // Format/3mf.cpp:1610
                let obj_type = extract_attr(line, "type");

                // Format/3mf.cpp:1620
                current_object = Some(ModelObjectBuilder::new(id, name, obj_type));
            } else {
                // Format/3mf.cpp:1668-1679
                if line.starts_with("<mesh>") && current_object.is_some() {
                    // Format/3mf.cpp:1670
                    let builder = current_object.as_mut().unwrap();
                    // Format/3mf.cpp:1671
                    i += 1;

                    // Format/3mf.cpp:1672-1678
                    while i < lines.len() && !lines[i].trim().starts_with("</mesh>") {
                        // Format/3mf.cpp:1673
                        let mesh_line = lines[i].trim();

                        // Format/3mf.cpp:1681-1693
                        if mesh_line.starts_with("<vertices>") {
                            // Format/3mf.cpp:1682
                            i += 1;
                            // Format/3mf.cpp:1684-1692
                            while i < lines.len() && !lines[i].trim().starts_with("</vertices>") {
                                // Format/3mf.cpp:1685
                                let vertex_line = lines[i].trim();
                                // Format/3mf.cpp:1694-1703
                                if vertex_line.starts_with("<vertex") {
                                    // Format/3mf.cpp:1698-1701
                                    if let Some(v) = parse_vertex(vertex_line) {
                                        // Format/3mf.cpp:1700
                                        builder.add_vertex(v);
                                    }
                                }
                                // Format/3mf.cpp:1691
                                i += 1;
                            }
                        } else {
                            // Format/3mf.cpp:1711-1743
                            if mesh_line.starts_with("<triangles>") {
                                // Format/3mf.cpp:1712
                                i += 1;
                                // Format/3mf.cpp:1714-1742
                                while i < lines.len()
                                    && !lines[i].trim().starts_with("</triangles>")
                                {
                                    // Format/3mf.cpp:1715
                                    let tri_line = lines[i].trim();
                                    // Format/3mf.cpp:1724-1743
                                    if tri_line.starts_with("<triangle") {
                                        // Format/3mf.cpp:1735-1738
                                        parse_triangle(tri_line, &vertices, builder);
                                    }
                                    // Format/3mf.cpp:1741
                                    i += 1;
                                }
                            }
                        }

                        // Format/3mf.cpp:1677
                        i += 1;
                    }
                } else {
                    // Format/3mf.cpp:206-233
                    if line.starts_with("<m:transform") && current_object.is_some() {
                        // Format/3mf.cpp:208
                        if let Some(transform_str) = extract_attr(line, "transform") {
                            // Format/3mf.cpp:210
                            let builder = current_object.as_mut().unwrap();
                            // Format/3mf.cpp:212-232
                            if let Some(transform) = parse_transform(&transform_str) {
                                // Format/3mf.cpp:230
                                object_transforms.insert(builder.id, transform);
                            }
                        }
                    } else {
                        // Format/3mf.cpp:1627-1665
                        if line.starts_with("</object>") {
                            // Format/3mf.cpp:1630-1665
                            if let Some(builder) = current_object.take() {
                                // Format/3mf.cpp:1660
                                object_builders.push(builder);
                            }
                        } else {
                            // Format/3mf.cpp:1802-1816
                            if line.starts_with("<item") {
                                // Format/3mf.cpp:1804
                                let object_id = extract_attr(line, "objectid")
                                    .and_then(|s| s.parse::<usize>().ok());
                                // Format/3mf.cpp:1806
                                let transform_str = extract_attr(line, "transform");

                                // Format/3mf.cpp:1808-1815
                                if let Some(obj_id) = object_id {
                                    // Format/3mf.cpp:1810-1814
                                    let transform = transform_str
                                        .as_ref()
                                        .and_then(|s| parse_transform(s))
                                        .unwrap_or_else(Transform3D::identity);

                                    // Format/3mf.cpp:1815
                                    object_transforms.insert(obj_id, transform);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Format/3mf.cpp:999
        i += 1;
    }

    // Format/3mf.cpp:1002
    if let Some(builder) = current_object {
        // Format/3mf.cpp:1003
        object_builders.push(builder);
    }

    // Format/3mf.cpp:1006-1016
    for builder in object_builders {
        // Format/3mf.cpp:1007
        if let Some(mesh) = builder.build_mesh() {
            // Format/3mf.cpp:1008
            let mut object = ModelObject::new(builder.name, mesh);

            // Format/3mf.cpp:1010-1014
            if let Some(transform) = object_transforms.get(&builder.id) {
                // Format/3mf.cpp:1011
                let position = Point3F::new(
                    transform.matrix[12],
                    transform.matrix[13],
                    transform.matrix[14],
                );
                // Format/3mf.cpp:1013
                object.add_instance(position);
            }

            // Format/3mf.cpp:1015
            model.add_object(object);
        }
    }

    // Format/3mf.cpp:1017
    if model.objects.is_empty() {
        // Format/3mf.cpp:1017
        return Err(Error::Mesh(
            "No valid objects found in 3MF file".to_string(),
        ));
    }

    // Format/3mf.cpp:1017
    Ok(model)
}

/// Parse a vertex element from 3MF XML
/// Format/3mf.cpp:1694-1703
fn parse_vertex(line: &str) -> Option<Point3F> {
    // Format/3mf.cpp:1698
    let x = extract_attr(line, "x")?.parse::<f64>().ok()?;
    // Format/3mf.cpp:1699
    let y = extract_attr(line, "y")?.parse::<f64>().ok()?;
    // Format/3mf.cpp:1700
    let z = extract_attr(line, "z")?.parse::<f64>().ok()?;
    // Format/3mf.cpp:1701
    Some(Point3F::new(x, y, z))
}

/// Parse a triangle element and add to builder
/// Format/3mf.cpp:1724-1743
fn parse_triangle(line: &str, vertices: &[Point3F], builder: &mut ModelObjectBuilder) {
    // Format/3mf.cpp:1730-1735
    if let (Some(v1), Some(v2), Some(v3)) = (
        extract_attr(line, "v1").and_then(|s| s.parse::<usize>().ok()),
        extract_attr(line, "v2").and_then(|s| s.parse::<usize>().ok()),
        extract_attr(line, "v3").and_then(|s| s.parse::<usize>().ok()),
    ) {
        // Format/3mf.cpp:1735-1738
        if v1 < vertices.len() && v2 < vertices.len() && v3 < vertices.len() {
            // Format/3mf.cpp:1737
            builder.add_triangle(vertices[v1], vertices[v2], vertices[v3]);
        }
    }
}

/// Parse a 3MF transform string into a Transform3D matrix
/// Format/3mf.cpp:206-233
fn parse_transform(transform_str: &str) -> Option<Transform3D> {
    // Format/3mf.cpp:210-215
    let values: Vec<f64> = transform_str
        .split_whitespace()
        .filter_map(|s| s.parse::<f64>().ok())
        .collect();

    // Format/3mf.cpp:217-232
    if values.len() == 12 {
        // Format/3mf.cpp:220-230
        Some(Transform3D {
            matrix: [
                // Format/3mf.cpp:221
                values[0], values[4], values[8], 0.0, // Format/3mf.cpp:224
                values[1], values[5], values[9], 0.0, // Format/3mf.cpp:227
                values[2], values[6], values[10], 0.0, // Format/3mf.cpp:230
                values[3], values[7], values[11], 1.0,
            ],
        })
    } else {
        // Format/3mf.cpp:232
        None
    }
}

/// Extract an XML attribute value by name from a tag string
/// Format/3mf.cpp:165-176
fn extract_attr(line: &str, attr_name: &str) -> Option<String> {
    // Format/3mf.cpp:167
    let pattern = format!("{}=\"", attr_name);
    // Format/3mf.cpp:169-175
    if let Some(start) = line.find(&pattern) {
        // Format/3mf.cpp:170
        let value_start = start + pattern.len();
        // Format/3mf.cpp:171-174
        if let Some(end) = line[value_start..].find('"') {
            // Format/3mf.cpp:173
            return Some(line[value_start..value_start + end].to_string());
        }
    }
    // Format/3mf.cpp:176
    None
}

// ============================================================================
// Model Object Builder
// ============================================================================

/// Builder for constructing ModelObjects during 3MF XML parsing
/// Format/3mf.cpp:391-658
struct ModelObjectBuilder {
    id: usize,
    name: String,
    obj_type: Option<String>,
    vertices: Vec<Point3F>,
    triangles: Vec<[Point3F; 3]>,
}

/// Implementation of the 3MF model object builder
/// Format/3mf.cpp:660-670
impl ModelObjectBuilder {
    // Create a new builder with given id, name, and optional type
    // Format/3mf.cpp:660-670
    fn new(id: usize, name: String, obj_type: Option<String>) -> Self {
        // Format/3mf.cpp:662-668
        Self {
            id,
            name,
            obj_type,
            vertices: Vec::new(),
            triangles: Vec::new(),
        }
    }

    /// Add a vertex to the builder's vertex list
    /// Format/3mf.cpp:1698-1701
    fn add_vertex(&mut self, v: Point3F) {
        // Format/3mf.cpp:1700
        self.vertices.push(v);
    }

    /// Add a triangle from three vertex positions
    /// Format/3mf.cpp:1735-1738
    fn add_triangle(&mut self, v1: Point3F, v2: Point3F, v3: Point3F) {
        // Format/3mf.cpp:1737
        self.triangles.push([v1, v2, v3]);
    }

    /// Build a TriangleMesh from accumulated vertices and triangles
    /// Format/3mf.cpp:1627-1665
    fn build_mesh(&self) -> Option<TriangleMesh> {
        // Format/3mf.cpp:1628
        if self.triangles.is_empty() {
            // Format/3mf.cpp:1629
            return None;
        }

        // Format/3mf.cpp:1632
        let mut vertices: Vec<Point3F> = Vec::new();
        // Format/3mf.cpp:1633
        let mut indices: Vec<u32> = Vec::new();

        // Format/3mf.cpp:1636-1650
        for tri in &self.triangles {
            // Format/3mf.cpp:1637-1649
            for v in tri {
                // Format/3mf.cpp:1638-1648
                let idx = vertices
                    .iter()
                    .position(|p| {
                        (p.x - v.x).abs() < 1e-6
                            && (p.y - v.y).abs() < 1e-6
                            && (p.z - v.z).abs() < 1e-6
                    })
                    .map(|i| i as u32)
                    .unwrap_or_else(|| {
                        // Format/3mf.cpp:1645-1647
                        let idx = vertices.len() as u32;
                        // Format/3mf.cpp:1646
                        vertices.push(*v);
                        idx
                    });
                // Format/3mf.cpp:1649
                indices.push(idx);
            }
        }

        // Format/3mf.cpp:1652-1656
        let triangles: Vec<crate::Triangle> = indices
            .chunks_exact(3)
            .map(|chunk| crate::Triangle::new(chunk[0], chunk[1], chunk[2]))
            .collect();

        // Format/3mf.cpp:1658-1663
        let mut mesh = TriangleMesh::new();
        // Format/3mf.cpp:1659
        for v in vertices {
            // Format/3mf.cpp:1660
            mesh.add_vertex(v);
        }
        // Format/3mf.cpp:1661
        for t in triangles {
            // Format/3mf.cpp:1662
            mesh.add_triangle(t);
        }

        // Format/3mf.cpp:1665
        Some(mesh)
    }
}

// ============================================================================
// 3D Model Generation
// ============================================================================

/// Generate 3D model XML content for 3MF export
/// Format/3mf.cpp:2514-2871
fn generate_3d_model(model: &Model) -> Result<String> {
    // Format/3mf.cpp:2516
    let mut xml = String::new();

    // Format/3mf.cpp:2518
    xml.push_str(r#"<?xml version="1.0" encoding="UTF-8"?>"#);
    // Format/3mf.cpp:2519
    xml.push('\n');
    // Format/3mf.cpp:2520-2522
    xml.push_str(r#"<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:m="http://schemas.microsoft.com/3dmanufacturing/material/2015/02">"#);
    // Format/3mf.cpp:2523
    xml.push('\n');

    // Format/3mf.cpp:2526
    xml.push_str("  <resources>\n");

    // Format/3mf.cpp:2528
    let mut object_vertex_counts: Vec<usize> = Vec::new();

    // Format/3mf.cpp:2574-2845
    for (obj_idx, obj) in model.objects.iter().enumerate() {
        // Format/3mf.cpp:2575
        let vertex_count = obj.mesh.vertex_count();
        // Format/3mf.cpp:2576
        object_vertex_counts.push(vertex_count);

        // Format/3mf.cpp:2578-2580
        xml.push_str(&format!(
            r#"    <object id="{}" name="{}" type="model">"#,
            obj_idx + 1,
            escape_xml(&obj.name)
        ));
        // Format/3mf.cpp:2581
        xml.push('\n');
        // Format/3mf.cpp:2582
        xml.push_str("      <mesh>\n");

        // Format/3mf.cpp:2680-2720
        xml.push_str("        <vertices>\n");
        // Format/3mf.cpp:2682
        let vertices = obj.mesh.vertices();
        // Format/3mf.cpp:2684-2695
        for v in vertices {
            // Format/3mf.cpp:2686-2692
            xml.push_str(&format!(
                r#"          <vertex x="{}" y="{}" z="{}" />"#,
                v.x, v.y, v.z
            ));
            // Format/3mf.cpp:2693
            xml.push('\n');
        }
        // Format/3mf.cpp:2695
        xml.push_str("        </vertices>\n");

        // Format/3mf.cpp:2700-2740
        xml.push_str("        <triangles>\n");
        // Format/3mf.cpp:2702
        let indices = obj.mesh.indices();
        // Format/3mf.cpp:2704-2735
        for tri in indices {
            // Format/3mf.cpp:2706-2732
            xml.push_str(&format!(
                r#"          <triangle v1="{}" v2="{}" v3="{}" />"#,
                tri.indices[0], tri.indices[1], tri.indices[2]
            ));
            // Format/3mf.cpp:2733
            xml.push('\n');
        }
        // Format/3mf.cpp:2735
        xml.push_str("        </triangles>\n");

        // Format/3mf.cpp:2740
        xml.push_str("      </mesh>\n");
        // Format/3mf.cpp:2742
        xml.push_str("    </object>\n");
    }

    // Format/3mf.cpp:2845
    xml.push_str("  </resources>\n");

    // Format/3mf.cpp:2850-2868
    xml.push_str("  <build>\n");
    // Format/3mf.cpp:2851
    let mut item_id = 1;
    // Format/3mf.cpp:2852-2866
    for (obj_idx, obj) in model.objects.iter().enumerate() {
        // Format/3mf.cpp:2853-2865
        for instance in &obj.instances {
            // Format/3mf.cpp:2855
            let transform = instance_to_transform(instance);
            // Format/3mf.cpp:2857-2864
            xml.push_str(&format!(
                r#"    <item objectid="{}" id="{}" transform="{}" />"#,
                obj_idx + 1,
                item_id,
                transform_to_string(&transform)
            ));
            // Format/3mf.cpp:2865
            xml.push('\n');
            // Format/3mf.cpp:2866
            item_id += 1;
        }
    }
    // Format/3mf.cpp:2868
    xml.push_str("  </build>\n");

    // Format/3mf.cpp:2870
    xml.push_str("</model>\n");

    // Format/3mf.cpp:2871
    Ok(xml)
}

/// Convert an Instance to a Transform3D matrix
/// Format/3mf.cpp:2645
fn instance_to_transform(instance: &Instance) -> Transform3D {
    // Format/3mf.cpp:2646
    let mut transform = Transform3D::identity();

    // Format/3mf.cpp:2648-2651
    transform = transform.translate(
        instance.position.x,
        instance.position.y,
        instance.position.z,
    );

    // Format/3mf.cpp:2653-2655
    if instance.rotation_z != 0.0 {
        // Format/3mf.cpp:2654
        transform = transform.rotate_z(instance.rotation_z.to_radians());
    }

    // Format/3mf.cpp:2657-2659
    if instance.scale[0] != 1.0 || instance.scale[1] != 1.0 || instance.scale[2] != 1.0 {
        // Format/3mf.cpp:2658
        transform = transform.scale(instance.scale[0], instance.scale[1], instance.scale[2]);
    }

    // Format/3mf.cpp:2661
    transform
}

/// Convert a Transform3D to a 3MF transform string
/// Format/3mf.cpp:2857-2864
fn transform_to_string(transform: &Transform3D) -> String {
    // Format/3mf.cpp:2858
    let m = transform.matrix;
    // Format/3mf.cpp:2859-2863
    format!(
        "{} {} {} {} {} {} {} {} {} {} {} {}",
        m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6], m[10], m[14]
    )
}

/// Escape special XML characters in a string
/// Format/3mf.cpp:2546
fn escape_xml(s: &str) -> String {
    // Format/3mf.cpp:2546
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}

/// Find vertex index in mesh (placeholder)
/// Format/3mf.cpp:2680-2845
fn find_vertex_index(_mesh: &TriangleMesh, _vertex: Point3F) -> usize {
    // Format/3mf.cpp:2680
    0
}

// ============================================================================
// ZIP Archive Utilities
// ============================================================================

/// Read a named entry from a ZIP archive as a UTF-8 string
/// Format/3mf.cpp:955-970
fn read_zip_entry<R: Read + std::io::Seek>(
    archive: &mut zip::ZipArchive<R>,
    name: &str,
) -> Result<String> {
    // Format/3mf.cpp:957-960
    let mut file = archive
        .by_name(name)
        .map_err(|e| Error::Mesh(format!("Failed to read {}: {}", name, e)))?;

    // Format/3mf.cpp:962-966
    let mut content = String::new();
    // Format/3mf.cpp:964
    file.read_to_string(&mut content)
        .map_err(|e| Error::Mesh(format!("Failed to read {} content: {}", name, e)))?;

    // Format/3mf.cpp:968
    Ok(content)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_attr() {
        let line = r#"<vertex x="1.0" y="2.0" z="3.0" />"#;
        assert_eq!(extract_attr(line, "x"), Some("1.0".to_string()));
        assert_eq!(extract_attr(line, "y"), Some("2.0".to_string()));
        assert_eq!(extract_attr(line, "z"), Some("3.0".to_string()));
        assert_eq!(extract_attr(line, "w"), None);
    }

    #[test]
    fn test_parse_transform() {
        let transform_str = "1 0 0 10 0 1 0 20 0 0 1 30";
        let transform = parse_transform(transform_str);
        assert!(transform.is_some());
    }

    #[test]
    fn test_escape_xml() {
        assert_eq!(escape_xml("<test>"), "&lt;test&gt;");
        assert_eq!(escape_xml("test & more"), "test &amp; more");
        assert_eq!(escape_xml("\"quoted\""), "&quot;quoted&quot;");
    }
}
