//! `adaptive_layer_merge` against AdaptiveLayerMerge.cpp at the pin.
//!
//! `tests/data/adaptive_layer_merge/vectors.txt` is the output of `gen.cpp` beside it, which
//! compiles the C++ file itself (the header comment there says how). Every case is compared
//! exactly: integers as they are, doubles and floats by their bits.
//!
//! Covered by the vectors: `AdaptiveLayerHeightRange::contains`, `adaptive_layer_height_range`,
//! `support_top_z_for_nozzle`, `adaptive_layer_band_size`, `has_multiple_used_nozzle_diameters`,
//! `build_adaptive_layer_bands` (and through it `cover_span_with_bands` / `cover_span_from_base`),
//! `pull_adaptive_band_top_to_material`, `split_adaptive_layer_span`.
//! The geometry functions (`adaptive_printable_pieces`, `adaptive_printable_core`,
//! `adaptive_suppressed_orphan_area`) need ClipperUtils and have plain tests at the end, with
//! cases read off the C++.

use slicer::adaptive_layer_merge::*;
use slicer::geometry::{ExPolygon, Point, Polygon};

const VECTORS: &str = include_str!("data/adaptive_layer_merge/vectors.txt");

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("integer")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double bits"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float bits"))
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "->");
    }
    fn heights(&mut self, table: &[f64]) -> Vec<f64> {
        let n = self.u();
        (0..n).map(|_| table[self.u()]).collect()
    }
    fn bytes(&mut self) -> Vec<u8> {
        let n = self.u();
        (0..n).map(|_| self.u() as u8).collect()
    }
}

/// Runs `check` over every line tagged `tag` and returns how many there were.
fn for_each(tag: &str, mut check: impl FnMut(&mut Tok, &[f64], &str)) -> usize {
    let mut table: Vec<f64> = Vec::new();
    let mut count = 0;
    for line in VECTORS.lines() {
        let mut tok = Tok(line.split_whitespace());
        let head = tok.s();
        if head == "T" {
            table = tok.0.by_ref().map(|t| f64::from_bits(u64::from_str_radix(t, 16).unwrap())).collect();
            assert_eq!(table.len(), 32);
        } else if head == tag {
            check(&mut tok, &table, line);
            count += 1;
        }
    }
    count
}

#[test]
fn layer_height_range_and_contains() {
    let n = for_each("RANGE", |t, _, line| {
        let nd = t.d();
        t.arrow();
        let got = adaptive_layer_height_range(nd);
        assert_eq!(got.min_height.to_bits(), t.d().to_bits(), "{line}");
        assert_eq!(got.max_height.to_bits(), t.d().to_bits(), "{line}");
    });
    assert_eq!(n, 40);
    let n = for_each("CONTAINS", |t, _, line| {
        let range = AdaptiveLayerHeightRange { min_height: t.d(), max_height: t.d() };
        let h = t.d();
        t.arrow();
        assert_eq!(range.contains(h), t.u() != 0, "{line}");
    });
    assert_eq!(n, 200);
}

#[test]
fn support_top_z() {
    let n = for_each("TOPZ", |t, _, line| {
        let nd = t.d();
        t.arrow();
        assert_eq!(support_top_z_for_nozzle(nd).to_bits(), t.f().to_bits(), "{line}");
    });
    assert_eq!(n, 49);
}

#[test]
fn band_size() {
    let n = for_each("BSIZE", |t, _, line| {
        let (nd, min_nd, base) = (t.d(), t.d(), t.d());
        t.arrow();
        assert_eq!(adaptive_layer_band_size(nd, min_nd, base), t.u(), "{line}");
    });
    assert_eq!(n, 800);
}

#[test]
fn multiple_used_nozzle_diameters() {
    let n = for_each("MULTI", |t, _, line| {
        let n = t.u();
        let ids: Vec<u32> = (0..n).map(|_| t.u() as u32).collect();
        let m = t.u();
        let diameters: Vec<f32> = (0..m).map(|_| t.f()).collect();
        t.arrow();
        // The same stand-in for nozzle_diameter_for_filament() as gen.cpp.
        let lookup = |filament_id: u32| {
            diameters[(if filament_id == 0 { 0 } else { filament_id as usize - 1 }) % diameters.len()]
        };
        assert_eq!(has_multiple_used_nozzle_diameters(&lookup, &ids), t.u() != 0, "{line}");
    });
    assert_eq!(n, 300);
}

#[test]
fn layer_bands() {
    let mut with_bands = 0;
    let n = for_each("BANDS", |t, table, line| {
        let band_size = t.u();
        let range = AdaptiveLayerHeightRange { min_height: t.d(), max_height: t.d() };
        let heights = t.heights(table);
        let tops = t.bytes();
        t.arrow();
        let got = build_adaptive_layer_bands(&heights, band_size, &range, &tops);
        let k = t.u();
        assert_eq!(got.len(), k, "{line}");
        for band in &got {
            assert_eq!((band.bottom_idx, band.top_idx), (t.u(), t.u()), "{line}");
            assert_eq!(band.merged_height.to_bits(), t.d().to_bits(), "{line}");
        }
        with_bands += usize::from(k > 0);
    });
    assert_eq!(n, 600);
    assert!(with_bands > 200, "only {with_bands} cases produce bands");
}

#[test]
fn pull_band_top_to_material() {
    let n = for_each("PULL", |t, table, line| {
        let mut band = AdaptiveLayerBand { bottom_idx: t.u(), top_idx: t.u(), merged_height: t.d() };
        let range = AdaptiveLayerHeightRange { min_height: t.d(), max_height: t.d() };
        let heights = t.heights(table);
        let material = t.bytes();
        t.arrow();
        let ret = pull_adaptive_band_top_to_material(&mut band, &heights, &range, &material);
        assert_eq!(ret, t.u() != 0, "{line}");
        assert_eq!((band.bottom_idx, band.top_idx), (t.u(), t.u()), "{line}");
        assert_eq!(band.merged_height.to_bits(), t.d().to_bits(), "{line}");
    });
    assert_eq!(n, 500);
}

#[test]
fn split_layer_span() {
    let n = for_each("SPLIT", |t, table, line| {
        let (bottom, top, preferred) = (t.u(), t.u(), t.u());
        let range = AdaptiveLayerHeightRange { min_height: t.d(), max_height: t.d() };
        let heights = t.heights(table);
        t.arrow();
        // Like gen.cpp: the vector is not empty on entry, the function clears it.
        let mut groups = vec![(7usize, 7usize)];
        let ret = split_adaptive_layer_span(&heights, bottom, top, preferred, &range, &mut groups);
        assert_eq!(ret, t.u() != 0, "{line}");
        let k = t.u();
        let want: Vec<(usize, usize)> = (0..k).map(|_| (t.u(), t.u())).collect();
        assert_eq!(groups, want, "{line}");
    });
    assert_eq!(n, 600);
}

// ---------------------------------------------------------------------------------------------
// The geometry functions. No C++ reference is built for these (they need ClipperUtils); the cases
// are read off AdaptiveLayerMerge.cpp:39-88.
// ---------------------------------------------------------------------------------------------

/// An axis-aligned rectangle, in millimetres.
fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> ExPolygon {
    let p = |x: f64, y: f64| Point::new((x * 1e5).round() as i64, (y * 1e5).round() as i64);
    ExPolygon::new(Polygon::from_points(vec![p(x0, y0), p(x1, y0), p(x1, y1), p(x0, y1)]))
}

fn area_mm2(pieces: &[ExPolygon]) -> f64 {
    pieces.iter().map(|p| p.area() * 1e-10).sum()
}

#[test]
fn printable_pieces_keeps_the_originals() {
    // 0.4 nozzle: min_area = 0.25 * 0.4 * 0.4 = 0.04 mm2, erosion 0.2 mm.
    let big = rect(0.0, 0.0, 5.0, 5.0);
    // Area 0.03 mm2: below min_area.
    let small = rect(10.0, 0.0, 10.1, 0.3);
    // Area 3 mm2, but 0.3 mm wide: nothing is left after eroding 0.2 mm from both sides.
    let thin = rect(20.0, 0.0, 20.3, 10.0);
    // 0.5 mm wide: 0.1 mm is left after the erosion, so it stays, as it was.
    let narrow = rect(30.0, 0.0, 30.5, 10.0);
    let kept = adaptive_printable_pieces(vec![big.clone(), small, thin, narrow.clone()], 0.4);
    assert_eq!(kept.len(), 2);
    assert_eq!(kept[0].contour.points(), big.contour.points());
    assert_eq!(kept[1].contour.points(), narrow.contour.points());
    // A nozzle of diameter 0 has min_area 0 and erosion 0: `erosion > 0.f` is false, every piece of
    // non-negative area stays.
    assert_eq!(adaptive_printable_pieces(vec![rect(0.0, 0.0, 0.01, 0.01)], 0.0).len(), 1);
    assert!(adaptive_printable_pieces(Vec::new(), 0.4).is_empty());
}

#[test]
fn printable_core_trims_the_tail() {
    // A 5 x 5 mm body with a 0.2 mm wide, 6 mm long tail: the piece passes the yes/no test, and the
    // opening by 0.2 mm removes the tail, into which no 0.4 bead fits.
    let p = |x: f64, y: f64| Point::new((x * 1e5).round() as i64, (y * 1e5).round() as i64);
    let piece = ExPolygon::new(Polygon::from_points(vec![
        p(0.0, 0.0),
        p(5.0, 0.0),
        p(5.0, 2.0),
        p(11.0, 2.0),
        p(11.0, 2.2),
        p(5.0, 2.2),
        p(5.0, 5.0),
        p(0.0, 5.0),
    ]));
    let core = adaptive_printable_core(vec![piece.clone()], 0.4);
    assert_eq!(core.len(), 1);
    let area = area_mm2(&core);
    assert!((area - 25.0).abs() < 0.05, "core area {area}");
    // The core lies within the piece it came from.
    assert!(area <= area_mm2(&[piece]));
    // Radius 0: the pieces come back as they are.
    let same = adaptive_printable_core(vec![rect(0.0, 0.0, 1.0, 1.0)], 0.0);
    assert_eq!(same.len(), 1);
    assert_eq!(same[0].contour.points(), rect(0.0, 0.0, 1.0, 1.0).contour.points());
    // A piece that fails the first test never reaches the opening.
    assert!(adaptive_printable_core(vec![rect(20.0, 0.0, 20.3, 10.0)], 0.4).is_empty());
}

#[test]
fn merge_patch_areas() {
    let mut patch = AdaptiveMergePatch::default();
    assert!(patch.area().is_empty());
    assert!(patch.suppressed_area().is_empty());
    assert!(!patch.has_suppressed_area());
    patch.set_area(vec![rect(0.0, 0.0, 1.0, 1.0)]);
    assert_eq!(patch.area().len(), 1);
    // An empty suppressed area still counts as set (`bool(m_suppressed_area)`).
    patch.set_suppressed_area(Vec::new());
    assert!(patch.has_suppressed_area());
    assert!(patch.suppressed_area().is_empty());
    // Copies share the geometry.
    let copy = patch.clone();
    assert!(std::ptr::eq(copy.area(), patch.area()));
}

#[test]
fn suppressed_orphan_area() {
    use slicer::surface::{Surface, SurfaceType};

    let orphan = |area: ExPolygon| {
        let mut patch = AdaptiveMergePatch::default();
        patch.group_top = true;
        patch.set_suppressed_area(vec![area]);
        patch
    };
    let band = |area: ExPolygon| {
        let mut patch = AdaptiveMergePatch::default();
        patch.group_top = true;
        patch.set_area(vec![area]);
        patch
    };

    // No region has an orphan: nothing is suppressed.
    let slices = vec![Surface::new(SurfaceType::Internal, rect(0.0, 0.0, 10.0, 10.0))];
    let plain = AdaptiveSuppressedOrphanRegion { adaptive_merge_patches: &[], slices: &slices };
    assert!(adaptive_suppressed_orphan_area(&[plain]).is_empty());

    // One orphan and nothing extruded: the union of the orphan footprint.
    let patches = vec![orphan(rect(0.0, 0.0, 4.0, 4.0))];
    let lost = AdaptiveSuppressedOrphanRegion { adaptive_merge_patches: &patches, slices: &[] };
    let area = adaptive_suppressed_orphan_area(&[lost]);
    assert_eq!(area.len(), 1);
    assert!((area_mm2(&area) - 16.0).abs() < 1e-9);

    // Another region still extrudes over half of it: a region without patches counts with its
    // slices, a region with patches with the area of each patch.
    let half = vec![Surface::new(SurfaceType::Internal, rect(0.0, 0.0, 2.0, 4.0))];
    let other = AdaptiveSuppressedOrphanRegion { adaptive_merge_patches: &[], slices: &half };
    let area = adaptive_suppressed_orphan_area(&[lost, other]);
    assert!((area_mm2(&area) - 8.0).abs() < 1e-9);
    let band_patches = vec![band(rect(0.0, 0.0, 4.0, 1.0))];
    // Its slices are not read once it has patches.
    let banded = AdaptiveSuppressedOrphanRegion { adaptive_merge_patches: &band_patches, slices: &slices };
    let area = adaptive_suppressed_orphan_area(&[lost, banded]);
    assert!((area_mm2(&area) - 12.0).abs() < 1e-9);

    // A patch that is not the group top, or that has an area, is not an orphan even with a
    // suppressed footprint set.
    let mut lower = orphan(rect(0.0, 0.0, 4.0, 4.0));
    lower.group_top = false;
    let lower = vec![lower];
    let region = AdaptiveSuppressedOrphanRegion { adaptive_merge_patches: &lower, slices: &[] };
    assert!(adaptive_suppressed_orphan_area(&[region]).is_empty());
}
