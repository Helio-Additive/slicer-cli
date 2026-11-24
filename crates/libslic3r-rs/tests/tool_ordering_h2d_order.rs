//! The per-layer filament order of a printer with two extruders (Bambu Lab H2D, the Majora job, 8
//! filaments, filament_map_mode "Auto For Flush") against the pin: ToolOrdering.cpp:3510-3548
//! (get_recommended_filament_maps :2511-2690) and :2844-2873 (execute_filament_ordering ->
//! reorder_filaments_for_minimum_flush_volume, ToolOrderUtils.cpp:1120-1297), through
//! `reorder_extruders_for_minimum_flush_volume_multi_extruder`, which the live export calls under
//! H2D_GROUP_ORDER.
//!
//! Data (tests/data/tool_ordering/h2d_majora_order.txt.gz): `C key value` the job's full
//! configuration as the port's export holds it (template options left out: the ordering reads
//! none), `L layer filaments` the LayerTools' filaments the ordering is handed (layer 0 in the
//! first-layer order, the others sorted), `E layer filaments` the tool changes of slicer_cli's
//! G-code for the same job, in order (`M620 S<n>A` per `; CHANGE_LAYER`; the first filament of the
//! print is loaded before the first layer change).
use slicer::config::ConfigBase;
use slicer::gcode::tool_ordering::{reorder_extruders_for_minimum_flush_volume_multi_extruder, ToolOrderingPrintConfig};
use std::io::Read;

#[test]
fn h2d_majora_layer_order_equals_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tool_ordering/h2d_majora_order.txt.gz");
    let mut s = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).expect("vectors")).read_to_string(&mut s).expect("gzip");
    let mut cfg = ToolOrderingPrintConfig::full_print_config();
    let hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let mut layers: Vec<Vec<u32>> = Vec::new();
    let mut expected: Vec<Vec<u32>> = Vec::new();
    let mut failed: Vec<(String, String)> = Vec::new();
    let nums = |v: &str| -> Vec<u32> { v.split_whitespace().map(|x| x.parse::<u32>().expect("filament")).collect() };
    for line in s.lines() {
        let mut it = line.splitn(3, '\t');
        let (tag, key, value) = (it.next().unwrap_or(""), it.next().unwrap_or(""), it.next().unwrap_or(""));
        match tag {
            // options the defaults do not know are left out, as a configuration load would
            // (an enum vector without a keys map panics in the port's deserializer: skipped likewise;
            // the ordering reads none of those)
            "C" => {
                let mut c = cfg.clone();
                let ok = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| c.set_deserialize_strict(key, value, false).is_ok()));
                if matches!(ok, Ok(true)) {
                    cfg = c;
                } else {
                    failed.push((key.to_string(), value.to_string()));
                }
            }
            "L" => layers.push(nums(value)),
            "E" => expected.push(nums(value)),
            _ => {}
        }
    }
    std::panic::set_hook(hook);
    eprintln!("not loaded: {failed:?}");
    // Those are enum vectors of one value per extruder / filament whose names the port cannot read
    // back (all entries equal here): their default value, repeated to the job's count.
    for (key, value) in &failed {
        let n = value.split(',').count();
        if let Ok(Some(opt)) = cfg.option_mut(key, false) {
            if let Some(v) = opt.as_vector_base_mut() {
                let _ = v.resize(n, None);
            }
        }
    }
    assert_eq!(layers.len(), 985);
    assert_eq!(expected.len(), 983);
    // prepare_ordering_context(reorder_first_layer = false): Print::process builds ToolOrdering(*this, -1).
    let (seqs, maps) = reorder_extruders_for_minimum_flush_volume_multi_extruder(&cfg, &layers, false).expect("grouping");
    // slicer_cli's header: filament_map = 1,2,2,1,2,2,2,1
    assert_eq!(maps, vec![0, 1, 1, 0, 1, 1, 1, 0]);
    assert_eq!(seqs.len(), layers.len());
    // The port prints two layers more than slicer_cli on this job (slicing); the first 983 are compared.
    let mut prev: Option<u32> = seqs[0].first().copied();
    let mut mismatches = Vec::new();
    for (i, exp) in expected.iter().enumerate() {
        let seq = &seqs[i];
        let changes: Vec<u32> = if !seq.is_empty() && Some(seq[0]) == prev { seq[1..].to_vec() } else { seq.clone() };
        if let Some(&last) = seq.last() {
            prev = Some(last);
        }
        if &changes != exp {
            mismatches.push((i, changes, exp.clone()));
        }
    }
    assert!(mismatches.is_empty(), "{} of {} layers differ, first {:?}", mismatches.len(), expected.len(), mismatches.first());
}
