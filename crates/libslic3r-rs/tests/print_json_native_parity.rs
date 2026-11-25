//! The JSON (de)serializers of the cached slicing data (Print.cpp:4500-5314, `print::print_json`)
//! against the pin's own. `tests/data/print_json/gen.cpp` builds random structures, writes each with
//! the pin's `to_json` (`T`), reads that text back with the pin's `from_json` / `extract_layer` /
//! `extract_support_layer` and writes the result again (`R`). Here the `T` text goes through the
//! port's from_json and to_json and the dump must equal the pin's `R` text byte for byte.

use serde_json::Value;
use slicer::extrusion_entity::{ExtrusionEntityCollection, ExtrusionPath, ExtrusionRole};
use slicer::geometry::{BoundingBox, ExPolygon, Polygon, Polyline};
use slicer::layer::{Layer, LayerRegion};
use slicer::print::faithful::{groupedVolumeSlices, SupportLayer};
use slicer::print::print_json::*;
use slicer::surface::{Surface, SurfaceType};
use std::io::Read;

fn vectors() -> String {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/print_json/vectors.txt.gz");
    let file = std::fs::File::open(path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn region_hashes(t: &Value) -> Vec<u64> {
    t["layer_regions"].as_array().unwrap().iter().map(|r| r["config_hash"].as_u64().unwrap()).collect()
}

/// The members export_cached_data writes for a layer (Print.cpp:5402-5460), as gen.cpp's layer_json.
fn layer_json(layer: &Layer, hashes: &[u64]) -> Value {
    let mut j = serde_json::Map::new();
    j.insert(JSON_LAYER_SLICED_POLYGONS.into(), Value::Array(layer.lslices.iter().map(to_value).collect()));
    j.insert(JSON_LAYER_SLLICED_BBOXES.into(), Value::Array(layer.lslices_bboxes.iter().map(to_value).collect()));
    j.insert(JSON_LAYER_OVERHANG_POLYGONS.into(), Value::Array(layer.loverhangs.iter().map(to_value).collect()));
    j.insert(JSON_LAYER_OVERHANG_BBOX.into(), to_value(&layer.loverhangs_bbox));
    let regions: Vec<Value> = layer
        .regions()
        .iter()
        .zip(hashes)
        .map(|(r, h)| to_value(&LayerRegionWithHash { layer_region: r, config_hash: *h }))
        .collect();
    j.insert(JSON_LAYER_REGIONS.into(), Value::Array(regions));
    Value::Object(j)
}

fn collection_json(c: &ExtrusionEntityCollection) -> Value {
    let mut j = serde_json::Map::new();
    j.insert(JSON_EXTRUSION_NO_SORT.into(), Value::from(c.no_sort));
    j.insert(JSON_EXTRUSION_ENTITY_TYPE.into(), Value::from(JSON_EXTRUSION_TYPE_COLLECTION));
    let mut entities = Vec::new();
    for e in &c.entities {
        let mut ej = Value::Null;
        let mut ep = Value::Array(Vec::new());
        if convert_extrusion_to_json(&mut ej, &mut ep, e) {
            entities.push(ej);
        }
    }
    j.insert(JSON_EXTRUSION_ENTITIES.into(), Value::Array(entities));
    Value::Object(j)
}

fn new_layer(i: usize, nreg: usize) -> Layer {
    let mut layer = Layer::new(i, 0, 0.2, 0.2 * (i as f64 + 1.0), 0.2 * i as f64 + 0.1);
    for k in 0..nreg {
        layer.add_region(LayerRegion::new(i, k));
    }
    layer
}

fn round_trip(kind: &str, case: usize, nreg: usize, t: &Value) -> Value {
    match kind {
        "bbox" => {
            let mut b = BoundingBox::new();
            BoundingBox::from_json(t, &mut b);
            to_value(&b)
        }
        "surface" => {
            let mut s = Surface::new(SurfaceType::Internal, ExPolygon::new(Polygon::new()));
            Surface::from_json(t, &mut s);
            to_value(&s)
        }
        "polyline" => {
            let mut p = Polyline::new();
            Polyline::from_json(t, &mut p);
            to_value(&p)
        }
        "path" => {
            let mut p = ExtrusionPath::new(ExtrusionRole::None);
            ExtrusionPath::from_json(t, &mut p);
            to_value(&p)
        }
        "entity" => {
            let mut c = ExtrusionEntityCollection::new();
            let ok = convert_extrusion_from_json(t, &mut c);
            let mut r = Value::Null;
            let mut rp = Value::Array(Vec::new());
            if ok && c.entities.len() == 1 {
                convert_extrusion_to_json(&mut r, &mut rp, &c.entities[0]);
            }
            r
        }
        "patch" => adaptive_merge_patch_to_json(&adaptive_merge_patch_from_json(t)),
        "group" => {
            let mut g = groupedVolumeSlices::default();
            groupedVolumeSlices::from_json(t, &mut g);
            to_value(&g)
        }
        "layer" => {
            let mut layer = new_layer(case, nreg);
            extract_layer(t, &mut layer).expect("extract_layer");
            layer_json(&layer, &region_hashes(t))
        }
        "support" => {
            let mut sl = SupportLayer::new(case, 0, 0, 0.2, 0.2 * (case as f64 + 1.0), 0.2 * case as f64 + 0.1);
            sl.layer = new_layer(case, nreg);
            extract_support_layer(t, &mut sl).expect("extract_support_layer");
            let mut r = layer_json(&sl.layer, &region_hashes(t));
            let islands: Vec<Value> = sl.support_islands.iter().map(to_value).collect();
            r[JSON_SUPPORT_LAYER_ISLANDS] = Value::Array(islands);
            r[JSON_SUPPORT_LAYER_FILLS] = collection_json(&sl.support_fills);
            r
        }
        other => panic!("unknown kind {other}"),
    }
}

#[test]
fn print_json_round_trips_match_the_pin() {
    let text = vectors();
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len() % 2, 0);
    let mut per_kind: std::collections::BTreeMap<String, usize> = Default::default();
    let mut case_of_kind: std::collections::BTreeMap<String, usize> = Default::default();
    let mut mismatches = Vec::new();
    for pair in lines.chunks(2) {
        let (tl, rl) = (pair[0], pair[1]);
        let mut it = tl.splitn(3, ' ');
        assert_eq!(it.next(), Some("T"));
        let kind = it.next().unwrap().to_string();
        let mut rest = it.next().unwrap();
        let mut nreg = 0usize;
        if kind == "layer" || kind == "support" {
            let (n, j) = rest.split_once(' ').unwrap();
            nreg = n.parse().unwrap();
            rest = j;
        }
        let expected = rl.splitn(3, ' ').nth(2).unwrap();
        let case = *case_of_kind.get(&kind).unwrap_or(&0);
        case_of_kind.insert(kind.clone(), case + 1);
        let t = parse(rest.as_bytes()).expect("parse T");
        let got = dump(&round_trip(&kind, case, nreg, &t));
        *per_kind.entry(kind.clone()).or_default() += 1;
        if got != expected {
            if mismatches.len() < 3 {
                let at = got.bytes().zip(expected.bytes()).position(|(a, b)| a != b).unwrap_or(0);
                let lo = at.saturating_sub(80);
                eprintln!(
                    "{kind} case {case}: first difference at byte {at}\n  port {}\n  pin  {}",
                    &got[lo..(at + 80).min(got.len())],
                    &expected[lo..(at + 80).min(expected.len())]
                );
            }
            mismatches.push((kind.clone(), case));
        }
    }
    eprintln!("cases per kind: {per_kind:?}");
    assert!(mismatches.is_empty(), "{} of {} cases differ: {:?}", mismatches.len(), lines.len() / 2, &mismatches[..mismatches.len().min(20)]);
    assert_eq!(lines.len() / 2, 660);
}
