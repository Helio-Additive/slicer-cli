//! Tier-1 Rust 3MF reader tests.
//!
//! These exercise the inline-mesh parser used by the `--engine rust` 3MF path
//! (`slicer::app_slice::slice_3mf_to_gcode`). They run as an integration test
//! (compiled against the crate's public API) so they are independent of the
//! crate's lib-internal test modules.

use slicer::app_slice::parse_3mf_model_xml;
use slicer::model::FacetsAnnotation;

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
    let mesh = parse_3mf_model_xml(INLINE_TETRA_MODEL)
        .expect("inline mesh should parse")
        .mesh;
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
    let mesh = parse_3mf_model_xml(ROTATED_Z90_MODEL)
        .expect("rotated mesh should parse")
        .mesh;
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
    let mesh = parse_3mf_model_xml(NEGATIVE_PART_MODEL)
        .expect("model should parse")
        .mesh;
    // Only the printable tetra (4 verts / 4 tris) — the type="other" triangle
    // at (100,100,100) must NOT be merged.
    assert_eq!(mesh.vertex_count(), 4, "negative volume leaked into merge");
    assert_eq!(mesh.triangle_count(), 4);
    assert!(
        mesh.vertices().iter().all(|v| v.x < 50.0),
        "found vertex from the type=\"other\" volume"
    );
}

/// The inline tetra with BambuStudio `paint_color` painting on two of the
/// four triangles (values from a real MakerWorld multicolour export).
const PAINTED_TETRA_MODEL: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
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
     <triangle v1="0" v2="1" v3="2" paint_color="4"/>
     <triangle v1="0" v2="1" v3="3"/>
     <triangle v1="0" v2="2" v3="3" paint_color="2C"/>
     <triangle v1="1" v2="2" v3="3"/>
    </triangles>
   </mesh>
  </object>
 </resources>
 <build>
  <item objectid="1"/>
 </build>
</model>"#;

#[test]
fn captures_paint_color_into_facets_annotation() {
    let parsed = parse_3mf_model_xml(PAINTED_TETRA_MODEL).expect("painted mesh should parse");
    assert_eq!(parsed.mesh.triangle_count(), 4);
    let facets = &parsed.mmu_facets;
    assert_eq!(facets.facet_count(), 2, "two painted triangles expected");
    // Round-trip through the FacetsAnnotation hex codec (Model.cpp:4267/4292):
    // what was stored for each triangle must decode back to the source string.
    assert_eq!(facets.get_triangle_as_string(0), "4");
    assert_eq!(facets.get_triangle_as_string(1), "", "unpainted triangle");
    assert_eq!(facets.get_triangle_as_string(2), "2C");
    assert_eq!(facets.get_triangle_as_string(3), "", "unpainted triangle");
}

#[test]
fn painted_states_decode_to_extruder_slots() {
    // Deserialize the painted tetra's annotation over its own mesh and check
    // the painted extruder slots come back. BambuStudio hex states: "4"
    // (0100 → leaf, state 01) = extruder 1; "2C" = a split/extended state
    // whose leaves resolve to extruder ≥1 — the decode must not panic and
    // must produce nonempty used_states.
    let parsed = parse_3mf_model_xml(PAINTED_TETRA_MODEL).expect("painted mesh should parse");
    let mut selector = slicer::triangle_selector::TriangleSelector::new(parsed.mesh, 0.0);
    selector.deserialize(
        &parsed.mmu_facets.data,
        false,
        slicer::triangle_selector::EnforcerBlockerType::EXTRUDER_MAX,
        slicer::triangle_selector::EnforcerBlockerType::NONE,
        slicer::triangle_selector::EnforcerBlockerType::NONE,
    );
    let states = selector.used_states();
    assert!(
        !states.is_empty(),
        "painted annotation must yield at least one painted extruder state"
    );
    // paint_color="4" is extruder slot 1 (state 0b01).
    assert!(
        states.iter().any(|s| s.0 == 1),
        "expected extruder 1 among painted states, got {states:?}"
    );
}

#[test]
fn facets_annotation_string_round_trip() {
    // Longer real-world strings (split-triangle states from the Majora 3MF).
    let samples = ["4", "8", "0C", "5C", "41C1C1C31C1C1C3", "2C2C42C2C32C2C3"];
    let mut fa = FacetsAnnotation::default();
    for (i, s) in samples.iter().enumerate() {
        fa.set_triangle_from_string(i as i32, s);
    }
    for (i, s) in samples.iter().enumerate() {
        assert_eq!(fa.get_triangle_as_string(i as i32), *s, "sample {i}");
    }
    assert_eq!(fa.get_triangle_as_string(99), "", "absent triangle decodes empty");
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

/// R703 — `Metadata/model_settings.config` is the ONLY place BambuStudio records
/// whether a `type="other"` object is a negative volume or a modifier; the
/// 3dmodel.model says only "other". This asserts the subtype parse, fixture-free.
///
/// Shape taken from Majora's real model_settings: object 1 is the printable
/// mask, objects 2-7 are the six `Connector-*` negative volumes.
#[test]
fn parses_negative_part_ids_from_model_settings() {
    let xml = r#"<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="1">
    <part id="1" subtype="normal_part">
      <metadata key="name" value="AI3M_full_mask (2).stl"/>
    </part>
    <part id="2" subtype="negative_part">
      <metadata key="name" value="Connector-1_A"/>
    </part>
    <part id="3" subtype="negative_part">
      <metadata key="name" value="Connector-2_A"/>
    </part>
    <part id="9" subtype="modifier_part">
      <metadata key="name" value="SomeModifier"/>
    </part>
  </object>
</config>"#;
    let ids = slicer::app_slice::parse_negative_part_ids_from_model_settings(xml);
    let mut got: Vec<u32> = ids.into_iter().collect();
    got.sort_unstable();
    // Only negative_part; normal_part and modifier_part must NOT be included —
    // merging a modifier as solid would be worse than omitting it.
    assert_eq!(got, vec![2, 3]);
}

#[test]
fn negative_part_parse_is_empty_without_the_subtype() {
    let xml = r#"<config><object id="1"><part id="1" subtype="normal_part"/></object></config>"#;
    assert!(slicer::app_slice::parse_negative_part_ids_from_model_settings(xml).is_empty());
}
