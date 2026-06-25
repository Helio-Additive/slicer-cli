# R63 reclass findings — root cause of fill-reclass knot (c), FULLY TRACED

Branch: L74-reclass. Both engines instrumented (rust + C++ references tree), RECLASS_DBG-gated.
The instrumentation walked the cascade from the fill stage all the way up to the mesh slicer.
Each step's native-vs-rust numbers below.

## SUMMARY (one line)
The dominant pz=0.6 ISI/Bridge deficit is NOT a fill-stage or reclassification bug. It is a MESH
SLICER bug: at LAYER 1 (pz=0.4) rust produces 8 SPURIOUS HOLES (~78mm² of phantom voids) that C++
does not. Those holes propagate up: layer-2 reads them as unsupported -> over-classifies ~78->290mm²
as stBottomBridge -> steals area from stInternalSolid -> the ISI leftover fragments into 244 narrow
pieces -> FillConcentric starves them. Bridge feature over (+14.94), ISI under (-60.22) both follow.

## THE TRACE (top of cascade -> down)

### STEP 1 — detect_narrow (fill stage). FAITHFUL, not the root.
pz=0.6: C++ = 1 stInternalSolid expoly, 466.96mm², narrow=0 -> KEPT (rectilinear).
        RUST = 244 stInternalSolid expolys, 193.98mm² total, ALL narrow -> Concentric (starved).
pz=4.2: surfaces match; rust's 2nd ISI expoly carries a spurious hole -> FLOAT vs C++ NARROW (minor).
detect_narrow is a correct 1:1 port; it narrow-routes rust's slivers correctly. Wrong INPUT shape.

### STEP 2 — fill_surfaces by type entering group_fills. Area conserved, PARTITION differs.
pz=0.6:  | type          | C++            | RUST                    |
         | Top           | 1 × 20.82 mm²  | 1 × 2.81 mm²            |
         | BottomBridge  | NONE           | 1 × 290.84 mm²          |
         | InternalSolid | 1 × 466.96 mm² | 244 × 193.98 mm² (frag) |
         | solid total   | 487.78 mm²     | 487.63 mm² (conserved)  |
Rust reclassifies ~291mm² of C++'s InternalSolid as BottomBridge. Same total, wrong split.

### STEP 3 — bottom-bridge support diff (detect_surfaces_type, PrintObject.cpp:1546). The mechanism.
bottom_bridge = opening_ex(diff_ex(layer_slices, LOWER_LAYER.lslices), offset).
pz=0.6: | quantity            | C++              | RUST                |
        | cur_slices_a        | 559.529          | 559.489  (MATCH)    |
        | lower_lslices_a     | 545.980          | 467.788  (-78.19)   |
        | bottom_diff_a       | 13.540 (1 ex)    | 91.701 (9 ex)       |
The lower-lslices deficit (78.19mm²) EXACTLY equals the bottom_diff excess (78.16mm²). 1:1.
[NB the C++ `offset` prints as 4199.9 (scaled) vs rust 0.042 (mm); both are no-ops on these shapes,
 not the discriminator — but the rust opening passes mm where C++ passes scaled. Flagged, not causal.]

### STEP 4 — lower layer (layer 1) lslices == its region slices in BOTH engines. So root is the SLICES.
pz=0.4: C++ slices_a 545.980 (46 surf) == lslices 545.980 (1 ex).
        RUST slices_a 467.791 (106 surf) == lslices 467.788 (2 ex).
lslices/make_slices union is faithful. The 78mm² lives in region.slices itself.

### STEP 5 — raw slices at ENTRY of detect_surfaces_type (before any reclassification). PINS THE LAYER.
| layer | pz    | C++ raw slices    | RUST raw slices   | Δ      |
|-------|-------|-------------------|-------------------|--------|
| 0     | 0.200 | 420.359 (2 surf)  | 414.731 (2 surf)  | -5.63  |
| 1     | 0.400 | 545.980 (1 surf)  | 467.788 (2 surf)  | -78.19 |
| 2     | 0.600 | 559.529 (1 surf)  | 559.489 (1 surf)  | -0.04 (MATCH) |
Layer 2 raw slices MATCH. The entire cascade is seeded by LAYER 1 being sliced wrong.

### STEP 6 — layer-1 slice geometry. THE BUG.
| engine | k | area    | holes | bbox |
|--------|---|---------|-------|------|
| C++    | 0 | 545.980 | 0     | [-25.97,-8.24]-[13.37,8.24] |
| RUST   | 0 | 459.220 | **8** | [-25.14,-8.24]-[14.19,8.24] |
| RUST   | 1 | 8.569   | 0     | [-7.06,-1.96]-[-4.55,1.90]  |
**Rust's layer-1 slice has 8 SPURIOUS HOLES (~78mm² of phantom voids) and a detached 8.57mm²
island; C++ slices the same layer as ONE clean hole-free 545.98mm² contour.**

## ROOT CAUSE (answers knot (c))
A MESH SLICER divergence: at the Benchy hull bottom (layer 1, pz=0.4) rust's slice generates 8
phantom holes that C++ does not. This is upstream of detect_surfaces_type, the fill stage, and the
reclassification. It is in make_slices / slice_to_region / make_loops+make_expolygons (loop assembly
& hole orientation/closing). Candidate (i) from the handoff (rust's expolygon SHAPE differs) is
CONFIRMED, and localized to the slicer, NOT the fill stage. Candidate (ii) is ruled out.

This CONTRADICTS the handoff's off-limits assumptions ("mesh slicer bit-faithful", "lslices
byte-identical", "detect_surfaces_type creates the CORRECT 91.7mm² bottom-bridge"). The measured
facts: layer-1 raw slices differ by 78mm² with 8 spurious holes; the 91.7mm² bottom-bridge born at
layer 2 is itself already wrong (C++ bottom_diff there is 13.54mm², 1 ex).

## VERDICT — clean bail (deliverable B)
NO faithful fill-stage fix can converge this gap. At pz=0.6 C++ does not use Concentric (region is
KEPT rectilinear) and the rust region is the wrong area+shape because of the upstream slicer holes.
Shipping the FillConcentric boundary-seed (b) + reclass tweak (c) would fill rust's WRONG fragmented
194mm² region — a coincidental aggregate nudge that cannot reach per-feature convergence, and cannot
remove the 290mm² over-Bridge (which is born in the slicer). I did NOT ship a fill-stage patch.

## THE NEXT FAITHFUL STEP (precise, actionable)
Fix the SLICER hole over-generation at the Benchy hull bottom. Concretely:
1. Reproduce in isolation: slice layer 1 (pz=0.4) and dump the loops/expolygons BEFORE union.
   Expected C++: one CCW contour, no holes. Rust: contour + 8 CW (hole) loops + a stray island.
2. The 8 holes are almost certainly mis-oriented or non-closed loops being kept as holes in
   make_expolygons (the loop-assembly / closing step). The prior commit 6529a52 already touched
   make_expolygons closing/offset units — re-examine that path: the closing radius / offset_expolygons
   sign that decides whether tiny inner loops collapse (C++) or survive as holes (rust).
3. Success check at this layer: rust layer-1 raw slice -> 1 expoly, 0 holes, 545.98mm². That alone
   should collapse the pz=0.6 BottomBridge (290->~13mm²), restore InternalSolid (194->~467mm², single
   coherent region, KEPT rectilinear), and remove the fragmentation. Re-measure per-feature after.

   SPECIFIC SUSPECT (unverified, for next agent): triangle_mesh_slicer.rs:make_expolygons (line 1296).
   At lines 1312-1313 offset_out/offset_in are computed WITH scale_() (SCALED units) and passed to
   offset_expolygons(). But commit 6529a52 established the geo-clipper offset_expolygons takes UNSCALED
   mm elsewhere (it removed a scale() that was collapsing contours). If offset_expolygons expects mm
   here too, then make_expolygons is double-scaling the closing radius (off by 1e5), so the offset2_ex
   close-open that should collapse tiny inner loops behaves wrong and leaves them as the 8 spurious
   holes. CHECK the unit convention of offset_expolygons and whether make_expolygons's scale() at
   1312-1313 is consistent with 6529a52's fix. Do NOT assume — A/B test the layer-1 slice hole count.
4. The pz=4.2 spurious-hole-on-2nd-ISI-expoly (FLOAT vs NARROW) is a separate, smaller follow-up of
   the same family (spurious holes in slices), likely fixed by the same slicer correction.

## R63.5 CORRECTION (main-session review of the above — supersedes the SUSPECT + reframes the root)
The STEP-1..6 both-engine measurement above is SOUND and stands. Three corrections to the analysis:

1. **The named SUSPECT (make_expolygons:1312-1313 scale()) is a CONFIRMED NO-OP — do NOT chase it.**
   At runtime closing_radius=0 (default, triangle_mesh_slicer.rs:1397 → passed at :1549). The
   `if closing_radius >= extra_offset` branch then computes offset_out=scale(0)=0, offset_in=-scale(0)=0
   → make_expolygons is a PURE UNION, no offset/close at all. The scale() can't generate holes. Commit
   6529a52 already established this exact line as a runtime no-op on main.

2. **The closing_radius=0.049 hypothesis (R58 finding A / R59 slicer-fix) is REFUTED BY MAGNITUDE.**
   Rust's 8 holes total ~86mm² (~10mm² each: C++ outer 545.98 − rust net 459.22). A 0.049mm
   morphological close seals holes ~0.01mm², not 10mm². So C++ does NOT slice these holes and then
   close them — C++'s raw slice genuinely lacks the cabin-floor openings at z=0.3. (Consistent with
   R59, which threaded 0.049 and the cascade did NOT trigger + outer-wall G1 regressed +371. Dead end.)

3. **ROOT (corrected) = MESH SLICER on-plane facet classification (F2) — R63 VINDICATES R61, not R62.**
   The holes are real ~10mm² geometry present in rust's slice and absent in C++'s, with make_expolygons
   a no-op → the divergence is in the RAW LOOPS from slice_facet/make_loops (the near-horizontal
   cabin-floor facets at z≈0.3, exact-f32 z==slice_z on-plane classification). This is EXACTLY R61's
   "DEFINITIVE root = mesh slicer on-plane facet classification". R62 ("slicer ruled out → fill
   reclassification") is the OUTLIER round: it inferred slicer-faithfulness from CODE READING and never
   measured native's li=1 hole count; R63's both-engine A/B (native=0 holes) overturns it empirically.

### BRANCH CAVEAT
L74-reclass is OFF 83d024b and does NOT contain 6529a52 (the make_expolygons unscaled-mm fix that is on
main). Slicer work should be based on MAIN (alex/libslic3r-parity-engine), not on L74-reclass.

### THE DECISIVE UNTAKEN MEASUREMENT (true next step — localizes the exact divergent facet)
Dump the RAW LOOPS (before union, before make_expolygons) at z=0.3 (layer 1) in BOTH engines:
slice_facet output / make_loops loops — count, areas, orientations, and the FACETS crossing/touching
z=0.3 at the cabin floor with their exact f32 vertex z. If C++'s raw loops already lack the 8 holes →
the bug is the f32 on-plane facet classification (which side of z==0.3 a near-horizontal facet lands).
That is deep F2 (Coord/precision-foundational), needs the C++-side facet dump to pin the ULP divergence.

## INSTRUMENTATION (all RECLASS_DBG-gated; REVERTED before handback)
- rust: crates/libslic3r-rs/src/fill/mod.rs (detect_narrow + fill_surfaces-by-type),
        crates/libslic3r-rs/src/print_object.rs (detect entry, L1 slice detail, bottom-diff, L1 lslices).
- C++ : references .../Fill/Fill.cpp and .../PrintObject.cpp (mirrored). Reverted via git checkout.
