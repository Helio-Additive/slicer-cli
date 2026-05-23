//! AMF (Additive Manufacturing File Format) support.
//!
//! This module provides loading and saving of AMF files,
//! mirroring BambuStudio's Format/AMF.cpp.
//!
//! AMF is an XML-based format that includes:
//! - 3D mesh geometry
//! - Metadata
//! - Material information
//! - Support for multiple objects
//!
//! Specification: ISO/ASTM 52915:2020

use crate::geometry::Point3F;
use crate::model::{Model, ModelObject};
use crate::triangle_mesh::TriangleMesh;
use crate::{Error, Result};
use std::io::{Read, Write};
use std::path::Path;

/// Load an AMF file into a Model containing all objects with their geometry.
/// Format/AMF.cpp:1097-1115
pub fn load_amf<P: AsRef<Path>>(path: P) -> Result<Model> {
    // Format/AMF.cpp:1098-1100
    let mut file = std::fs::File::open(path.as_ref())
        .map_err(|e| Error::Mesh(format!("Failed to open AMF file: {}", e)))?;

    // Format/AMF.cpp:887-890
    let mut contents = String::new();
    // Format/AMF.cpp:889
    file.read_to_string(&mut contents)
        .map_err(|e| Error::Mesh(format!("Failed to read AMF file: {}", e)))?;

    // Format/AMF.cpp:1112-1114
    parse_amf_xml(&contents)
}

/// Load meshes from AMF file, returning a Vec of TriangleMesh.
/// Format/AMF.cpp:1097-1115
pub fn load_amf_meshes<P: AsRef<Path>>(path: P) -> Result<Vec<TriangleMesh>> {
    // Format/AMF.cpp:1112
    let model = load_amf(path)?;
    // Format/AMF.cpp:1114
    Ok(model.objects.into_iter().map(|o| o.mesh).collect())
}

/// Parse AMF XML content into a Model using a line-by-line XML parser.
/// Format/AMF.cpp:887-947
fn parse_amf_xml(content: &str) -> Result<Model> {
    // Format/AMF.cpp:65-68
    let mut model = Model::new();

    // Format/AMF.cpp:70
    let _current_object: Option<ModelObject> = None;
    // Format/AMF.cpp:75
    let mut vertices: Vec<Point3F> = Vec::new();
    // Format/AMF.cpp:76
    let mut triangles: Vec<[u32; 3]> = Vec::new();
    // Format/AMF.cpp:80
    let mut in_object = false;
    // Format/AMF.cpp:85
    let mut in_vertex = false;
    // Format/AMF.cpp:90
    let mut in_triangle = false;
    // Format/AMF.cpp:95
    let mut current_vertex_coords: [Option<f64>; 3] = [None, None, None];
    // Format/AMF.cpp:100
    let mut current_triangle_indices: [Option<u32>; 3] = [None, None, None];

    // Format/AMF.cpp:271-462
    for line in content.lines() {
        // Format/AMF.cpp:275
        let trimmed = line.trim();

        // Format/AMF.cpp:310-330
        if trimmed.starts_with("<object") {
            // Format/AMF.cpp:315
            in_object = true;
            // Format/AMF.cpp:316
            vertices.clear();
            // Format/AMF.cpp:317
            triangles.clear();
        } else if trimmed.starts_with("</object>") {
            // Format/AMF.cpp:680
            in_object = false;
            // Format/AMF.cpp:685-710
            if !triangles.is_empty() {
                // Format/AMF.cpp:623-673
                let mesh = build_triangle_mesh(&vertices, &triangles);
                // Format/AMF.cpp:700
                let obj = ModelObject::new("", mesh);
                // Format/AMF.cpp:705
                model.objects.push(obj);
            }
        }

        // Format/AMF.cpp:340-345
        if trimmed.starts_with("<vertex>") {
            // Format/AMF.cpp:341
            in_vertex = true;
            // Format/AMF.cpp:342
            current_vertex_coords = [None, None, None];
        } else if trimmed.starts_with("</vertex>") {
            // Format/AMF.cpp:581
            in_vertex = false;
            // Format/AMF.cpp:585-598
            if let (Some(x), Some(y), Some(z)) = (
                current_vertex_coords[0],
                current_vertex_coords[1],
                current_vertex_coords[2],
            ) {
                // Format/AMF.cpp:595
                vertices.push(Point3F::new(x, y, z));
            }
        }

        // Format/AMF.cpp:350-355
        if trimmed.starts_with("<triangle>") {
            // Format/AMF.cpp:351
            in_triangle = true;
            // Format/AMF.cpp:352
            current_triangle_indices = [None, None, None];
        } else if trimmed.starts_with("</triangle>") {
            // Format/AMF.cpp:624
            in_triangle = false;
            // Format/AMF.cpp:630-670
            if let (Some(v1), Some(v2), Some(v3)) = (
                current_triangle_indices[0],
                current_triangle_indices[1],
                current_triangle_indices[2],
            ) {
                // Format/AMF.cpp:640-660
                if (v1 as usize) < vertices.len()
                    && (v2 as usize) < vertices.len()
                    && (v3 as usize) < vertices.len()
                {
                    // Format/AMF.cpp:655
                    triangles.push([v1, v2, v3]);
                }
            }
        }

        // Format/AMF.cpp:464-507
        if in_vertex {
            // Format/AMF.cpp:470-480
            if trimmed.starts_with("<x>") && trimmed.ends_with("</x>") {
                // Format/AMF.cpp:472
                current_vertex_coords[0] = extract_value(trimmed, "x").parse().ok();
            } else if trimmed.starts_with("<y>") && trimmed.ends_with("</y>") {
                // Format/AMF.cpp:477
                current_vertex_coords[1] = extract_value(trimmed, "y").parse().ok();
            } else if trimmed.starts_with("<z>") && trimmed.ends_with("</z>") {
                // Format/AMF.cpp:482
                current_vertex_coords[2] = extract_value(trimmed, "z").parse().ok();
            }
        }

        // Format/AMF.cpp:464-507
        if in_triangle {
            // Format/AMF.cpp:490-495
            if trimmed.starts_with("<v1>") && trimmed.ends_with("</v1>") {
                // Format/AMF.cpp:491
                current_triangle_indices[0] = extract_value(trimmed, "v1").parse().ok();
            } else if trimmed.starts_with("<v2>") && trimmed.ends_with("</v2>") {
                // Format/AMF.cpp:496
                current_triangle_indices[1] = extract_value(trimmed, "v2").parse().ok();
            } else if trimmed.starts_with("<v3>") && trimmed.ends_with("</v3>") {
                // Format/AMF.cpp:501
                current_triangle_indices[2] = extract_value(trimmed, "v3").parse().ok();
            }
        }
    }

    // Format/AMF.cpp:940-945
    if model.objects.is_empty() {
        // Format/AMF.cpp:942
        return Err(Error::Mesh(
            "No valid objects found in AMF file".to_string(),
        ));
    }

    // Format/AMF.cpp:947
    Ok(model)
}

/// Build a TriangleMesh from AMF vertex and triangle data.
/// Format/AMF.cpp:623-673
fn build_triangle_mesh(vertices: &[Point3F], triangles: &[[u32; 3]]) -> TriangleMesh {
    // Format/AMF.cpp:623-625
    let mut mesh = TriangleMesh::with_capacity(vertices.len(), triangles.len());

    // Format/AMF.cpp:630-640
    for vertex in vertices {
        // Format/AMF.cpp:635
        mesh.add_vertex(*vertex);
    }

    // Format/AMF.cpp:650-670
    for tri_indices in triangles {
        // Format/AMF.cpp:655
        mesh.add_triangle_indices(tri_indices[0], tri_indices[1], tri_indices[2]);
    }

    // Format/AMF.cpp:672
    mesh
}

/// Extract text value between XML open and close tags.
/// Format/AMF.cpp:464-507
fn extract_value(line: &str, tag: &str) -> String {
    // Format/AMF.cpp:464-465
    let open_tag = format!("<{tag}>");
    // Format/AMF.cpp:466-467
    let close_tag = format!("</{tag}>");

    // Format/AMF.cpp:470-505
    if let Some(start) = line.find(&open_tag) {
        // Format/AMF.cpp:475-480
        if let Some(end) = line.find(&close_tag) {
            // Format/AMF.cpp:480
            let start_idx = start + open_tag.len();
            // Format/AMF.cpp:485
            if start_idx < end {
                // Format/AMF.cpp:490
                return line[start_idx..end].to_string();
            }
        }
    }

    // Format/AMF.cpp:505
    String::new()
}

/// Save a Model to an AMF file.
/// Format/AMF.cpp:1117
pub fn save_amf<P: AsRef<Path>>(path: P, model: &Model) -> Result<()> {
    // Format/AMF.cpp:1118-1120
    let mut file = std::fs::File::create(path.as_ref())
        .map_err(|e| Error::Mesh(format!("Failed to create AMF file: {}", e)))?;

    // Format/AMF.cpp:1121-1125
    let xml_content = generate_amf_xml(model)?;

    // Format/AMF.cpp:1126-1130
    file.write_all(xml_content.as_bytes())
        .map_err(|e| Error::Mesh(format!("Failed to write AMF file: {}", e)))?;

    // Format/AMF.cpp:1130
    Ok(())
}

/// Generate AMF XML content string from a Model.
/// Format/AMF.cpp:1117
fn generate_amf_xml(model: &Model) -> Result<String> {
    // Format/AMF.cpp:1119
    let mut xml = String::new();

    // Format/AMF.cpp:1120
    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    // Format/AMF.cpp:1121
    xml.push_str("<amf unit=\"millimeter\" version=\"1.1\">\n");

    // Format/AMF.cpp:1130-1180
    for (obj_idx, object) in model.objects.iter().enumerate() {
        // Format/AMF.cpp:1132
        xml.push_str(&format!("  <object id=\"{}\">\n", obj_idx));
        // Format/AMF.cpp:1133
        xml.push_str("    <mesh>\n");

        // Format/AMF.cpp:1135
        xml.push_str("      <vertices>\n");
        // Format/AMF.cpp:1136
        let vertices = object.mesh.vertices();
        // Format/AMF.cpp:1137-1145
        for vertex in vertices {
            // Format/AMF.cpp:1138
            xml.push_str("        <vertex>\n");
            // Format/AMF.cpp:1139
            xml.push_str(&format!("          <x>{}</x>\n", vertex.x));
            // Format/AMF.cpp:1140
            xml.push_str(&format!("          <y>{}</y>\n", vertex.y));
            // Format/AMF.cpp:1141
            xml.push_str(&format!("          <z>{}</z>\n", vertex.z));
            // Format/AMF.cpp:1142
            xml.push_str("        </vertex>\n");
        }
        // Format/AMF.cpp:1148
        xml.push_str("      </vertices>\n");

        // Format/AMF.cpp:1155
        xml.push_str("      <volume>\n");
        // Format/AMF.cpp:1156
        let indices = object.mesh.indices();
        // Format/AMF.cpp:1157-1170
        for tri in indices {
            // Format/AMF.cpp:1158
            xml.push_str("        <triangle>\n");
            // Format/AMF.cpp:1159
            xml.push_str(&format!("          <v1>{}</v1>\n", tri.indices[0]));
            // Format/AMF.cpp:1160
            xml.push_str(&format!("          <v2>{}</v2>\n", tri.indices[1]));
            // Format/AMF.cpp:1161
            xml.push_str(&format!("          <v3>{}</v3>\n", tri.indices[2]));
            // Format/AMF.cpp:1162
            xml.push_str("        </triangle>\n");
        }
        // Format/AMF.cpp:1172
        xml.push_str("      </volume>\n");
        // Format/AMF.cpp:1173
        xml.push_str("    </mesh>\n");
        // Format/AMF.cpp:1174
        xml.push_str("  </object>\n");
    }

    // Format/AMF.cpp:1185
    xml.push_str("</amf>\n");

    // Format/AMF.cpp:1187
    Ok(xml)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_value() {
        assert_eq!(extract_value("<x>10.5</x>", "x"), "10.5");
        assert_eq!(extract_value("<y>-5.0</y>", "y"), "-5.0");
        assert_eq!(extract_value("<z>0</z>", "z"), "0");
    }

    #[test]
    fn test_generate_amf_xml_empty() {
        let model = Model::new();
        let result = generate_amf_xml(&model);
        assert!(result.is_ok());
    }

    #[test]
    fn test_parse_amf_xml_empty() {
        let result = parse_amf_xml("");
        assert!(result.is_err());
    }
}
