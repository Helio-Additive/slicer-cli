//! Standard 3MF file format handler
//!
//! C++ Reference:
//! - Format/3mf.hpp
//! - Format/3mf.cpp
//!
//! Handles loading and saving of standard 3MF files (OPC package with
//! 3D model XML, relationships, and optional config/thumbnail data).

use crate::{Error, Result};
use std::collections::HashMap;
use std::io::{Read, Write};
use std::path::Path;

// ---------------------------------------------------------------------------
// Version constants (from 3mf.cpp)
// ---------------------------------------------------------------------------

/// Current version of the 3MF format saved by this implementation
pub const VERSION_3MF: u32 = 1;
/// Maximum compatible version we can load
pub const VERSION_3MF_COMPATIBLE: u32 = 2;
/// Metadata key for version in .model file
pub const SLIC3RPE_3MF_VERSION: &str = "slic3rpe:Version3mf";

/// SLA support points format version
pub const SUPPORT_POINTS_FORMAT_VERSION: u32 = 1;
/// Drain holes format version
pub const DRAIN_HOLES_FORMAT_VERSION: u32 = 1;

// Painting gizmos data versions
pub const FDM_SUPPORTS_PAINTING_VERSION: u32 = 1;
pub const SEAM_PAINTING_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// Internal constants
// ---------------------------------------------------------------------------

const MODEL_FILE: &str = "3D/3dmodel.model";
const CONTENT_TYPES_FILE: &str = "[Content_Types].xml";
const RELATIONSHIPS_FILE: &str = "_rels/.rels";
const SLIC3R_CONFIG_FILE: &str = "Metadata/Slic3r_PE_model.config";
const SLIC3R_PRINT_CONFIG: &str = "Metadata/Slic3r_PE.config";

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Prusa file parser: checks if a 3MF file originates from PrusaSlicer
/// Format/3mf.hpp: PrusaFileParser
#[derive(Debug, Clone)]
pub struct PrusaFileParser {
    from_prusa: bool,
    is_application_key: bool,
}

impl PrusaFileParser {
    pub fn new() -> Self {
        PrusaFileParser {
            from_prusa: false,
            is_application_key: false,
        }
    }

    /// Check if a 3MF file was saved by PrusaSlicer
    /// 3mf.hpp: check_3mf_from_prusa
    pub fn check_3mf_from_prusa(&mut self, filename: &str) -> bool {
        let zip_path = Path::new(filename);
        if !zip_path.exists() {
            return false;
        }

        let file = match std::fs::File::open(zip_path) {
            Ok(f) => f,
            Err(_) => return false,
        };
        let mut archive = match zip::ZipArchive::new(file) {
            Ok(a) => a,
            Err(_) => return false,
        };

        // Look for the model file and parse it for Prusa metadata
        if let Ok(mut model_file) = archive.by_name(MODEL_FILE) {
            let mut content = String::new();
            if model_file.read_to_string(&mut content).is_ok() {
                self.parse_for_prusa_marker(&content);
            }
        }

        self.from_prusa
    }

    /// Handle element start for Prusa detection
    fn start_element_handler(&mut self, name: &str, attributes: &HashMap<String, String>) {
        if name == "metadata" {
            if let Some(attr_name) = attributes.get("name") {
                if attr_name == "Application" || attr_name == "application" {
                    self.is_application_key = true;
                }
            }
        }
    }

    /// Handle character data for Prusa detection
    fn characters_handler(&mut self, text: &str) {
        if self.is_application_key {
            if text.contains("PrusaSlicer") || text.contains("Slic3r") {
                self.from_prusa = true;
            }
            self.is_application_key = false;
        }
    }

    /// Parse XML content looking for Prusa markers
    fn parse_for_prusa_marker(&mut self, content: &str) {
        use quick_xml::events::Event;
        use quick_xml::reader::Reader;

        let mut reader = Reader::from_str(content);
        reader.trim_text(true);
        let mut buf = Vec::new();

        loop {
            match reader.read_event_into(&mut buf) {
                Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                    let name = String::from_utf8_lossy(e.name().as_ref()).to_string();
                    let mut attrs = HashMap::new();
                    for attr in e.attributes().flatten() {
                        let key = String::from_utf8_lossy(attr.key.as_ref()).to_string();
                        let val = String::from_utf8_lossy(&attr.value).to_string();
                        attrs.insert(key, val);
                    }
                    self.start_element_handler(&name, &attrs);
                }
                Ok(Event::Text(ref t)) => {
                    let text = String::from_utf8_lossy(t.as_ref()).to_string();
                    self.characters_handler(&text);
                }
                Ok(Event::Eof) => break,
                Err(_) => break,
                _ => {}
            }
            buf.clear();
        }
    }
}

/// Dynamic print configuration (key-value pairs)
#[derive(Debug, Clone)]
pub struct DynamicPrintConfig {
    pub values: HashMap<String, String>,
}

impl DynamicPrintConfig {
    pub fn new() -> Self {
        DynamicPrintConfig {
            values: HashMap::new(),
        }
    }

    pub fn set(&mut self, key: &str, value: &str) {
        self.values.insert(key.to_string(), value.to_string());
    }

    pub fn get(&self, key: &str) -> Option<&str> {
        self.values.get(key).map(|s| s.as_str())
    }
}

/// Simple model for 3MF I/O
#[derive(Debug, Clone)]
pub struct Model {
    pub objects: Vec<ModelObject>,
    pub metadata: HashMap<String, String>,
}

impl Model {
    pub fn new() -> Self {
        Model {
            objects: Vec::new(),
            metadata: HashMap::new(),
        }
    }
}

/// Model object containing mesh data
#[derive(Debug, Clone)]
pub struct ModelObject {
    pub name: String,
    pub vertices: Vec<[f32; 3]>,
    pub indices: Vec<[i32; 3]>,
}

impl ModelObject {
    pub fn new() -> Self {
        ModelObject {
            name: String::new(),
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }
}

/// Thumbnail data
#[derive(Debug, Clone)]
pub struct ThumbnailData {
    pub width: u32,
    pub height: u32,
    pub pixels: Vec<u8>,
}

impl ThumbnailData {
    pub fn new() -> Self {
        ThumbnailData {
            width: 0,
            height: 0,
            pixels: Vec::new(),
        }
    }
}

/// Configuration substitution context for handling config migrations
#[derive(Debug, Clone)]
pub struct ConfigSubstitutionContext {
    pub substitutions: Vec<(String, String, String)>,
}

impl ConfigSubstitutionContext {
    pub fn new() -> Self {
        ConfigSubstitutionContext {
            substitutions: Vec::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// XML Helpers
// ---------------------------------------------------------------------------

/// Get string attribute value from XML attributes map
/// 3mf.hpp: get_attribute_value_string
pub fn get_attribute_value_string(attributes: &HashMap<String, String>, key: &str) -> String {
    attributes.get(key).cloned().unwrap_or_default()
}

/// Parse XML character handler callback (used in SAX-style parsing)
/// 3mf.hpp: characters_handler
pub fn characters_handler(text: &str) -> String {
    text.trim().to_string()
}

/// Create and return a PrusaFileParser
/// 3mf.hpp: prusa_file_parser
pub fn prusa_file_parser() -> PrusaFileParser {
    PrusaFileParser::new()
}

// ---------------------------------------------------------------------------
// Core I/O functions
// ---------------------------------------------------------------------------

/// Load a standard 3MF file into the given model and config
/// 3mf.cpp: load_3mf
pub fn load_3mf(
    path: &str,
    config: &mut DynamicPrintConfig,
    _config_substitutions: &mut ConfigSubstitutionContext,
    model: &mut Model,
    check_version: bool,
) -> Result<bool> {
    let zip_path = Path::new(path);
    if !zip_path.exists() {
        return Err(Error::IO(format!("File not found: {}", path)));
    }

    let file = std::fs::File::open(zip_path)
        .map_err(|e| Error::IO(format!("Failed to open 3MF: {}", e)))?;
    let mut archive = zip::ZipArchive::new(file)
        .map_err(|e| Error::IO(format!("Failed to read ZIP archive: {}", e)))?;

    // Read the 3D model
    if let Ok(mut model_file) = archive.by_name(MODEL_FILE) {
        let mut content = String::new();
        model_file
            .read_to_string(&mut content)
            .map_err(|e| Error::IO(format!("Failed to read model: {}", e)))?;
        parse_model_xml(&content, model, check_version)?;
    } else {
        return Err(Error::IO("No 3D model found in 3MF file".to_string()));
    }

    // Read config files
    for i in 0..archive.len() {
        let mut file = match archive.by_index(i) {
            Ok(f) => f,
            Err(_) => continue,
        };
        let name = file.name().to_string();

        if name == SLIC3R_PRINT_CONFIG || name == SLIC3R_CONFIG_FILE {
            let mut content = String::new();
            if file.read_to_string(&mut content).is_ok() {
                parse_config_content(&content, config);
            }
        }
    }

    Ok(true)
}

/// Save a model and config to a standard 3MF file
/// 3mf.cpp: store_3mf
pub fn store_3mf(
    path: &str,
    model: &Model,
    config: Option<&DynamicPrintConfig>,
    _fullpath_sources: bool,
    thumbnail_data: Option<&ThumbnailData>,
    _zip64: bool,
) -> Result<bool> {
    let file = std::fs::File::create(path)
        .map_err(|e| Error::IO(format!("Failed to create 3MF: {}", e)))?;
    let mut zip = zip::ZipWriter::new(file);

    let options =
        zip::write::FileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    // Write content types
    zip.start_file(CONTENT_TYPES_FILE, options)
        .map_err(|e| Error::IO(format!("ZIP error: {}", e)))?;
    write!(
        zip,
        r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"#
    )
    .map_err(|e| Error::IO(format!("Write error: {}", e)))?;

    // Write relationships
    zip.start_file(RELATIONSHIPS_FILE, options)
        .map_err(|e| Error::IO(format!("ZIP error: {}", e)))?;
    write!(
        zip,
        r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"#
    )
    .map_err(|e| Error::IO(format!("Write error: {}", e)))?;

    // Write 3D model
    zip.start_file(MODEL_FILE, options)
        .map_err(|e| Error::IO(format!("ZIP error: {}", e)))?;
    let xml = generate_model_xml(model);
    zip.write_all(xml.as_bytes())
        .map_err(|e| Error::IO(format!("Write error: {}", e)))?;

    // Write config
    if let Some(config) = config {
        zip.start_file(SLIC3R_PRINT_CONFIG, options)
            .map_err(|e| Error::IO(format!("ZIP error: {}", e)))?;
        let config_str = generate_config_content(config);
        zip.write_all(config_str.as_bytes())
            .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
    }

    // Write thumbnail if provided
    if let Some(thumb) = thumbnail_data {
        if !thumb.pixels.is_empty() {
            zip.start_file("Metadata/thumbnail.png", options)
                .map_err(|e| Error::IO(format!("ZIP error: {}", e)))?;
            zip.write_all(&thumb.pixels)
                .map_err(|e| Error::IO(format!("Write error: {}", e)))?;
        }
    }

    zip.finish()
        .map_err(|e| Error::IO(format!("Failed to finalize ZIP: {}", e)))?;

    Ok(true)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Parse 3MF model XML
fn parse_model_xml(content: &str, model: &mut Model, check_version: bool) -> Result<()> {
    use quick_xml::events::Event;
    use quick_xml::reader::Reader;

    let mut reader = Reader::from_str(content);
    reader.trim_text(true);

    let mut current_object: Option<ModelObject> = None;
    let mut in_vertices = false;
    let mut in_triangles = false;
    let mut buf = Vec::new();
    let mut current_metadata_key = String::new();

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) | Ok(Event::Empty(ref e)) => {
                let _is_empty = matches!(reader.read_event_into(&mut Vec::new()), _);
                match e.name().as_ref() {
                    b"object" => {
                        let mut obj = ModelObject::new();
                        for attr in e.attributes().flatten() {
                            if attr.key.as_ref() == b"name" {
                                obj.name = String::from_utf8_lossy(&attr.value).to_string();
                            }
                        }
                        current_object = Some(obj);
                    }
                    b"vertices" => in_vertices = true,
                    b"triangles" => in_triangles = true,
                    b"vertex" => {
                        if in_vertices {
                            if let Some(ref mut obj) = current_object {
                                let mut x = 0.0f32;
                                let mut y = 0.0f32;
                                let mut z = 0.0f32;
                                for attr in e.attributes().flatten() {
                                    match attr.key.as_ref() {
                                        b"x" => {
                                            x = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0.0)
                                        }
                                        b"y" => {
                                            y = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0.0)
                                        }
                                        b"z" => {
                                            z = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0.0)
                                        }
                                        _ => {}
                                    }
                                }
                                obj.vertices.push([x, y, z]);
                            }
                        }
                    }
                    b"triangle" => {
                        if in_triangles {
                            if let Some(ref mut obj) = current_object {
                                let mut v1 = 0i32;
                                let mut v2 = 0i32;
                                let mut v3 = 0i32;
                                for attr in e.attributes().flatten() {
                                    match attr.key.as_ref() {
                                        b"v1" => {
                                            v1 = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0)
                                        }
                                        b"v2" => {
                                            v2 = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0)
                                        }
                                        b"v3" => {
                                            v3 = String::from_utf8_lossy(&attr.value)
                                                .parse()
                                                .unwrap_or(0)
                                        }
                                        _ => {}
                                    }
                                }
                                obj.indices.push([v1, v2, v3]);
                            }
                        }
                    }
                    b"metadata" => {
                        for attr in e.attributes().flatten() {
                            if attr.key.as_ref() == b"name" {
                                current_metadata_key =
                                    String::from_utf8_lossy(&attr.value).to_string();
                            }
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::Text(ref t)) => {
                if !current_metadata_key.is_empty() {
                    let value = String::from_utf8_lossy(t.as_ref()).to_string();
                    model.metadata.insert(current_metadata_key.clone(), value);

                    // Check version if requested
                    if check_version && current_metadata_key == SLIC3RPE_3MF_VERSION {
                        if let Ok(ver) = String::from_utf8_lossy(t.as_ref()).parse::<u32>() {
                            if ver > VERSION_3MF_COMPATIBLE {
                                log::warn!(
                                    "3MF version {} is newer than supported {}",
                                    ver,
                                    VERSION_3MF_COMPATIBLE
                                );
                            }
                        }
                    }
                    current_metadata_key.clear();
                }
            }
            Ok(Event::End(ref e)) => match e.name().as_ref() {
                b"object" => {
                    if let Some(obj) = current_object.take() {
                        model.objects.push(obj);
                    }
                }
                b"vertices" => in_vertices = false,
                b"triangles" => in_triangles = false,
                b"metadata" => {
                    current_metadata_key.clear();
                }
                _ => {}
            },
            Ok(Event::Eof) => break,
            Err(e) => {
                return Err(Error::IO(format!("XML parse error: {}", e)));
            }
            _ => {}
        }
        buf.clear();
    }

    Ok(())
}

/// Generate 3MF model XML
fn generate_model_xml(model: &Model) -> String {
    let mut xml = String::new();
    xml.push_str(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
"#,
    );

    // Write metadata
    xml.push_str(&format!(
        "  <metadata name=\"{}\">{}</metadata>\n",
        SLIC3RPE_3MF_VERSION, VERSION_3MF
    ));
    for (key, value) in &model.metadata {
        if key != SLIC3RPE_3MF_VERSION {
            xml.push_str(&format!(
                "  <metadata name=\"{}\">{}</metadata>\n",
                key, value
            ));
        }
    }

    xml.push_str("  <resources>\n");
    for (i, obj) in model.objects.iter().enumerate() {
        xml.push_str(&format!(
            "    <object id=\"{}\" type=\"model\" name=\"{}\">\n",
            i + 1,
            obj.name
        ));
        xml.push_str("      <mesh>\n        <vertices>\n");
        for v in &obj.vertices {
            xml.push_str(&format!(
                "          <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>\n",
                v[0], v[1], v[2]
            ));
        }
        xml.push_str("        </vertices>\n        <triangles>\n");
        for t in &obj.indices {
            xml.push_str(&format!(
                "          <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>\n",
                t[0], t[1], t[2]
            ));
        }
        xml.push_str("        </triangles>\n      </mesh>\n    </object>\n");
    }

    xml.push_str("  </resources>\n  <build>\n");
    for (i, _) in model.objects.iter().enumerate() {
        xml.push_str(&format!("    <item objectid=\"{}\"/>\n", i + 1));
    }
    xml.push_str("  </build>\n</model>\n");

    xml
}

/// Parse config content (key = value lines)
fn parse_config_content(content: &str, config: &mut DynamicPrintConfig) {
    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        if let Some(pos) = line.find('=') {
            let key = line[..pos].trim();
            let value = line[pos + 1..].trim();
            config.set(key, value);
        }
    }
}

/// Generate config content for export
fn generate_config_content(config: &DynamicPrintConfig) -> String {
    let mut content = String::new();
    content.push_str("# generated by slicer-rs\n");
    let mut keys: Vec<&String> = config.values.keys().collect();
    keys.sort();
    for key in keys {
        if let Some(value) = config.values.get(key) {
            content.push_str(&format!("{} = {}\n", key, value));
        }
    }
    content
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_prusa_file_parser() {
        let mut parser = PrusaFileParser::new();
        assert!(!parser.from_prusa);

        // Test with Prusa-like XML content
        let xml = r#"<?xml version="1.0"?>
<model>
  <metadata name="Application">PrusaSlicer 2.5</metadata>
</model>"#;
        parser.parse_for_prusa_marker(xml);
        assert!(parser.from_prusa);
    }

    #[test]
    fn test_prusa_file_parser_non_prusa() {
        let mut parser = PrusaFileParser::new();
        let xml = r#"<?xml version="1.0"?>
<model>
  <metadata name="Application">BambuStudio</metadata>
</model>"#;
        parser.parse_for_prusa_marker(xml);
        assert!(!parser.from_prusa);
    }

    #[test]
    fn test_get_attribute_value_string() {
        let mut attrs = HashMap::new();
        attrs.insert("name".to_string(), "test".to_string());
        assert_eq!(get_attribute_value_string(&attrs, "name"), "test");
        assert_eq!(get_attribute_value_string(&attrs, "missing"), "");
    }

    #[test]
    fn test_model_roundtrip() {
        let mut model = Model::new();
        let obj = ModelObject {
            name: "TestObj".to_string(),
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            indices: vec![[0, 1, 2]],
        };
        model.objects.push(obj);

        let xml = generate_model_xml(&model);
        let mut loaded = Model::new();
        assert!(parse_model_xml(&xml, &mut loaded, false).is_ok());
        assert_eq!(loaded.objects.len(), 1);
        assert_eq!(loaded.objects[0].name, "TestObj");
        assert_eq!(loaded.objects[0].vertices.len(), 3);
    }

    #[test]
    fn test_config_roundtrip() {
        let mut config = DynamicPrintConfig::new();
        config.set("layer_height", "0.2");
        config.set("wall_count", "3");

        let content = generate_config_content(&config);
        let mut loaded = DynamicPrintConfig::new();
        parse_config_content(&content, &mut loaded);

        assert_eq!(loaded.get("layer_height"), Some("0.2"));
        assert_eq!(loaded.get("wall_count"), Some("3"));
    }
}
