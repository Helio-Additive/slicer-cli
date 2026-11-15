//! `support_merge_plan` against SupportMergePlan.cpp at the pin.
//!
//! `tests/data/support_merge_plan/vectors.txt` is the output of `gen.cpp` beside it, which compiles
//! SupportMergePlan.cpp and AdaptiveLayerMerge.cpp themselves against stand-ins for PrintObject and
//! the config (the header comment there says how, and describes the stream). For every object the
//! plan `make_support_merge_plan` returns is compared with the one the C++ returned, then every
//! query method is run on a plan read from the file and compared: integers as they are, doubles by
//! their bits.

use std::collections::BTreeMap;

use slicer::adaptive_layer_merge::{AdaptiveLayerBand, AdaptiveLayerHeightRange};
use slicer::support_merge_plan::*;

const VECTORS: &str = include_str!("data/support_merge_plan/vectors.txt");

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn tag(&mut self, tag: &str) {
        assert_eq!(self.s(), tag);
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("unsigned")
    }
    fn i(&mut self) -> i32 {
        self.s().parse().expect("integer")
    }
    fn b(&mut self) -> bool {
        self.u() != 0
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double bits"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float bits"))
    }
    fn doubles(&mut self) -> Vec<f64> {
        let n = self.u();
        (0..n).map(|_| self.d()).collect()
    }

    fn plan(&mut self, tag: &str) -> SupportMergePlan {
        self.tag(tag);
        let mut plan = SupportMergePlan::default();
        plan.enabled = self.b();
        plan.interface_fold_enabled = self.b();
        self.tag("PZ");
        plan.layer_print_z = self.doubles();
        self.tag("LH");
        plan.layer_heights = self.doubles();
        self.tag("CT");
        let n = self.u();
        let table: Vec<Candidate> = (0..n)
            .map(|_| Candidate {
                filament_id: self.i(),
                nozzle_diameter: self.d(),
                window: AdaptiveLayerHeightRange { min_height: self.d(), max_height: self.d() },
                band_size: self.u(),
            })
            .collect();
        for tag in ["CPL", "ICPL"] {
            self.tag(tag);
            let layers = self.u();
            let list: Vec<Vec<Candidate>> = (0..layers)
                .map(|_| {
                    let n = self.u();
                    (0..n).map(|_| table[self.u()]).collect()
                })
                .collect();
            if tag == "CPL" {
                plan.candidates_per_layer = list;
            } else {
                plan.interface_candidates_per_layer = list;
            }
        }
        self.tag("FB");
        let mut bands = BTreeMap::new();
        for _ in 0..self.u() {
            let filament_id = self.i();
            let n = self.u();
            let ladder: Vec<AdaptiveLayerBand> = (0..n)
                .map(|_| AdaptiveLayerBand { bottom_idx: self.u(), top_idx: self.u(), merged_height: self.d() })
                .collect();
            bands.insert(filament_id, ladder);
        }
        plan.fixed_bands_per_filament = bands;
        self.tag("SL");
        let n = self.u();
        plan.slabs = (0..n)
            .map(|_| Slab {
                bottom_idx: self.u(),
                top_idx: self.u(),
                top_z: self.d(),
                height: self.d(),
                filament_id: self.i(),
                nozzle_diameter: self.d(),
                bottom_anchor: self.b(),
            })
            .collect();
        self.tag("SI");
        let n = self.u();
        plan.slab_index_per_layer = (0..n).map(|_| self.i()).collect();
        plan
    }
}

/// Every float of the plan by its bits: `{:?}` of a double is the shortest text that reads back as
/// the same double, and it keeps the sign of zero.
fn text(plan: &SupportMergePlan) -> String {
    format!("{plan:#?}")
}

fn ids(candidates: &[Candidate]) -> Vec<i32> {
    candidates.iter().map(|c| c.filament_id).collect()
}

#[test]
fn make_plan_and_queries() {
    let mut t = Tok(VECTORS.split_whitespace());
    t.tag("N");
    let num_objects = t.u();
    assert_eq!(num_objects, 52);
    let (mut enabled, mut with_slabs, mut queries, mut hits) = (0, 0, 0, [0usize; 6]);
    for obj in 0..num_objects {
        t.tag("OBJ");
        let uses_multiple_nozzle_diameters = t.b();
        let (grid_nozzle, base_height) = (t.d(), t.d());
        let (support_filament, support_interface_filament) = (t.i(), t.i());
        t.tag("ND");
        let n = t.u();
        let diameters: Vec<f32> = (0..n).map(|_| t.f()).collect();
        t.tag("SOL");
        let n = t.u();
        let soluble: Vec<bool> = (0..n).map(|_| t.b()).collect();
        t.tag("VOL");
        let n = t.u();
        let model_part_extruders: Vec<Vec<i32>> = (0..n)
            .map(|_| {
                let n = t.u();
                (0..n).map(|_| t.i()).collect()
            })
            .collect();
        t.tag("LAYERS");
        let n = t.u();
        let layers: Vec<SupportMergePlanLayer> = (0..n)
            .map(|_| {
                let (print_z, height, empty) = (t.d(), t.d(), t.b());
                let regions = (0..t.u())
                    .map(|_| SupportMergePlanRegion {
                        slices_empty: t.b(),
                        perimeters_empty: t.b(),
                        wall_filament: t.i(),
                        sparse_infill_filament: t.i(),
                        solid_infill_filament: t.i(),
                        fill_filament_slots: {
                            let n = t.u();
                            (0..n).map(|_| t.i()).collect()
                        },
                    })
                    .collect();
                SupportMergePlanLayer { print_z, height, empty, regions }
            })
            .collect();

        // The same stand-ins as gen.cpp.
        let nozzle_diameter_for_filament = |filament_id: u32| {
            diameters[(if filament_id == 0 { 0 } else { filament_id as usize - 1 }) % diameters.len()]
        };
        let filament_soluble = |idx: usize| if idx < soluble.len() { soluble[idx] } else { soluble[0] };
        let object = SupportMergePlanObject {
            uses_multiple_nozzle_diameters,
            layers,
            grid_nozzle,
            base_height,
            nozzle_diameter_for_filament: &nozzle_diameter_for_filament,
            filament_soluble: &filament_soluble,
            model_part_extruders,
            support_filament,
            support_interface_filament,
        };
        let got = make_support_merge_plan(&object);
        let want = t.plan("PLAN");
        assert_eq!(text(&got), text(&want), "object {obj}: make_support_merge_plan");
        enabled += usize::from(got.enabled);
        with_slabs += usize::from(!got.slabs.is_empty());

        let plan = t.plan("MUT");
        t.tag("Q");
        for q in 0..t.u() {
            let (top_z, height) = (t.d(), t.d());
            let candidate = {
                let filament_id = t.i();
                let band_size = t.u();
                // fixed_band_height_at() reads the filament id and the band size only.
                Candidate { filament_id, band_size, ..Candidate::default() }
            };
            let layer_idx = t.u();
            t.tag("->");
            let at = format!("object {obj} query {q} top_z {top_z} height {height}");
            let mut list = |t: &mut Tok| -> Vec<i32> {
                let n = t.u();
                (0..n).map(|_| t.i()).collect()
            };
            let want = list(&mut t);
            assert_eq!(ids(&plan.candidates_in_span(top_z - height, top_z)), want, "{at}: candidates_in_span");
            hits[0] += usize::from(!want.is_empty());
            let want = list(&mut t);
            assert_eq!(ids(&plan.candidates_for_slab(top_z, height)), want, "{at}: candidates_for_slab");
            let c = plan.coarsest_for_slab(top_z, height);
            assert_eq!((c.filament_id, c.nozzle_diameter.to_bits()), (t.i(), t.d().to_bits()), "{at}: coarsest_for_slab");
            hits[1] += usize::from(c.valid());
            let c = plan.finest_interface_for_slab(top_z, height);
            assert_eq!(
                (c.filament_id, c.nozzle_diameter.to_bits()),
                (t.i(), t.d().to_bits()),
                "{at}: finest_interface_for_slab"
            );
            hits[2] += usize::from(c.valid());
            assert_eq!(plan.max_interface_height(top_z, height).to_bits(), t.d().to_bits(), "{at}: max_interface_height");
            assert_eq!(plan.max_height_for_slab(top_z, height).to_bits(), t.d().to_bits(), "{at}: max_height_for_slab");
            assert_eq!(plan.base_height_at(top_z).to_bits(), t.d().to_bits(), "{at}: base_height_at");
            let h = plan.fixed_band_height_at(&candidate, top_z);
            assert_eq!(h.to_bits(), t.d().to_bits(), "{at}: fixed_band_height_at");
            hits[3] += usize::from(h > 0.0);
            let mut band_height = -1.0;
            let c = plan.coarsest_band_containing(top_z, &mut band_height);
            assert_eq!(
                (c.filament_id, c.nozzle_diameter.to_bits(), band_height.to_bits()),
                (t.i(), t.d().to_bits(), t.d().to_bits()),
                "{at}: coarsest_band_containing"
            );
            hits[4] += usize::from(c.valid());
            let slab = plan.slab_for_layer(layer_idx);
            let got = match slab {
                None => -1,
                Some(slab) => plan.slabs.iter().position(|s| std::ptr::eq(s, slab)).unwrap() as i32,
            };
            assert_eq!(got, t.i(), "{at}: slab_for_layer({layer_idx})");
            hits[5] += usize::from(got >= 0);
            queries += 1;
        }
    }
    assert!(t.0.next().is_none(), "tokens left over");
    assert_eq!(queries, 52 * 20);
    // The vectors are not all the trivial answer.
    assert!(enabled >= 40 && with_slabs >= 20, "enabled {enabled}, with slabs {with_slabs}");
    assert!(hits.iter().all(|h| *h >= 100), "queries with a non-trivial answer: {hits:?}");
}

#[test]
fn defaults() {
    // SupportMergePlan.hpp:38-41, 57, 103
    let candidate = Candidate::default();
    assert_eq!((candidate.filament_id, candidate.band_size), (0, 1));
    assert!(!candidate.valid());
    assert!(!Candidate { filament_id: 1, ..candidate }.valid());
    assert!(Candidate { filament_id: 1, nozzle_diameter: 0.4, ..candidate }.valid());
    let plan = SupportMergePlan::default();
    assert!(!plan.enabled && plan.interface_fold_enabled);
    assert!(plan.slab_for_layer(0).is_none());
}
