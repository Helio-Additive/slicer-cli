//! Tier-1 Rust 3MF reader tests.
//!
//! These exercise the inline-mesh parser used by the `--engine rust` 3MF path
//! (`slicer::app_slice::slice_3mf_to_gcode`). They run as an integration test
//! (compiled against the crate's public API) so they are independent of the
//! crate's lib-internal test modules.

use slicer::app_slice::parse_3mf_model_xml;

/// A minimal single-file 3MF `3dmodel.model` with an inline tetrahedron mesh
/// (4 vertices, 4 triangles) referenced by one build item — the shape the
/// Tier-1 reader supports.
const INLINE_TETRA_MODEL: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <object id="1" type="model">
   <mesh>
    <vertices>
     <vertex x="0" y="0" z="0"/>
     <vertex x="10" y="0" z="0"/>
     <vertex x="0" y="10" z="0"/>
     <vertex x="0" y="0" z="10"/>
    </vertices>
    <triangles>
     <triangle v1="0" v2="1" v3="2"/>
     <triangle v1="0" v2="1" v3="3"/>
     <triangle v1="0" v2="2" v3="3"/>
     <triangle v1="1" v2="2" v3="3"/>
    </triangles>
   </mesh>
  </object>
 </resources>
 <build>
  <item objectid="1"/>
 </build>
</model>"#;

/// A production-extension `3dmodel.model`: the only object references an
/// external part via `p:path`, so there is no inline mesh in this XML.
const EXTERNAL_COMPONENT_MODEL: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter"
       xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
       xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">
 <resources>
  <object id="3" type="model">
   <components>
    <component p:path="/3D/Objects/object_1.model" objectid="1"/>
   </components>
  </object>
 </resources>
 <build>
  <item objectid="3"/>
 </build>
</model>"#;

#[test]
fn parses_inline_single_file_mesh() {
    let mesh = parse_3mf_model_xml(INLINE_TETRA_MODEL).expect("inline mesh should parse");
    assert_eq!(mesh.vertex_count(), 4);
    assert_eq!(mesh.triangle_count(), 4);
}

/// A single triangle placed by a build item that rotates +90° about Z.
/// 3MF stores the transform COLUMN-major, so the 12 values are the images of
/// the X, Y, Z basis vectors followed by the translation:
///   X↦(0,1,0)  Y↦(-1,0,0)  Z↦(0,0,1)  t=(0,0,0)
/// Under a correct (column-major) reading, vertex (1,0,0) ↦ (0,1,0). The old
/// row-major reading applied the transpose (the inverse rotation) and produced
/// (0,-1,0) — this test locks in the fix.
const ROTATED_Z90_MODEL: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <object id="1" type="model">
   <mesh>
    <vertices>
     <vertex x="1" y="0" z="0"/>
     <vertex x="0" y="1" z="0"/>
     <vertex x="0" y="0" z="1"/>
    </vertices>
    <triangles>
     <triangle v1="0" v2="1" v3="2"/>
    </triangles>
   </mesh>
  </object>
 </resources>
 <build>
  <item objectid="1" transform="0 1 0 -1 0 0 0 0 1 0 0 0"/>
 </build>
</model>"#;

#[test]
fn applies_build_item_rotation_column_major() {
    let mesh = parse_3mf_model_xml(ROTATED_Z90_MODEL).expect("rotated mesh should parse");
    let v = mesh.vertices();
    assert_eq!(v.len(), 3);
    let approx = |a: f64, b: f64| (a - b).abs() < 1e-9;
    // (1,0,0) ↦ (0,1,0)
    assert!(approx(v[0].x, 0.0) && approx(v[0].y, 1.0) && approx(v[0].z, 0.0), "v0 = {:?}", v[0]);
    // (0,1,0) ↦ (-1,0,0)
    assert!(approx(v[1].x, -1.0) && approx(v[1].y, 0.0) && approx(v[1].z, 0.0), "v1 = {:?}", v[1]);
    // (0,0,1) ↦ (0,0,1)
    assert!(approx(v[2].x, 0.0) && approx(v[2].y, 0.0) && approx(v[2].z, 1.0), "v2 = {:?}", v[2]);
}

/// A build item referencing a `type="model"` parent whose components point at
/// one printable tetra (object 1, `type="model"`) and one negative volume
/// (object 2, `type="other"` — how BambuStudio stores negative_part /
/// modifier volumes in the core XML). Only object 1's geometry may be merged;
/// unioning the "other" volume as positive solid was the pre-fix behavior.
const NEGATIVE_PART_MODEL: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <object id="1" type="model">
   <mesh>
    <vertices>
     <vertex x="0" y="0" z="0"/>
     <vertex x="10" y="0" z="0"/>
     <vertex x="0" y="10" z="0"/>
     <vertex x="0" y="0" z="10"/>
    </vertices>
    <triangles>
     <triangle v1="0" v2="1" v3="2"/>
     <triangle v1="0" v2="1" v3="3"/>
     <triangle v1="0" v2="2" v3="3"/>
     <triangle v1="1" v2="2" v3="3"/>
    </triangles>
   </mesh>
  </object>
  <object id="2" type="other">
   <mesh>
    <vertices>
     <vertex x="100" y="100" z="100"/>
     <vertex x="110" y="100" z="100"/>
     <vertex x="100" y="110" z="100"/>
    </vertices>
    <triangles>
     <triangle v1="0" v2="1" v3="2"/>
    </triangles>
   </mesh>
  </object>
  <object id="3" type="model">
   <components>
    <component objectid="1"/>
    <component objectid="2"/>
   </components>
  </object>
 </resources>
 <build>
  <item objectid="3"/>
 </build>
</model>"#;

#[test]
fn skips_negative_and_other_typed_objects() {
    let mesh = parse_3mf_model_xml(NEGATIVE_PART_MODEL).expect("model should parse");
    // Only the printable tetra (4 verts / 4 tris) — the type="other" triangle
    // at (100,100,100) must NOT be merged.
    assert_eq!(mesh.vertex_count(), 4, "negative volume leaked into merge");
    assert_eq!(mesh.triangle_count(), 4);
    assert!(
        mesh.vertices().iter().all(|v| v.x < 50.0),
        "found vertex from the type=\"other\" volume"
    );
}

#[test]
fn rejects_production_extension_with_clear_error() {
    let err = parse_3mf_model_xml(EXTERNAL_COMPONENT_MODEL)
        .expect_err("production-extension 3MF has no inline mesh");
    let msg = format!("{err}");
    assert!(
        msg.contains("production extension"),
        "error should name the production extension, got: {msg}"
    );
    assert!(
        msg.contains("--engine native"),
        "error should point at --engine native, got: {msg}"
    );
}
