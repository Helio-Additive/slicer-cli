# R63 reclass findings — STEP 1 both-engine measurement at detect_narrow

Branch: L74-reclass. Both engines instrumented at detect_narrow_internal_solid_infill
(rust crates/libslic3r-rs/src/fill/mod.rs group_fills; C++ references .../Fill/Fill.cpp:453).
Gated on RECLASS_DBG env + print_z in {0.6, 4.2}.

## lower_internal_areas (the "float" gate input)
| pz  | engine | count | area_mm2 | bbox |
|-----|--------|-------|----------|------|
| 0.6 | C++    | 0     | 0.000    | (none — lower layer is layer 1; its fill_surfaces have no stInternal/stInternalVoid) |
| 0.6 | RUST   | 11    | 0.039    | [-24.39,-2.94]-[10.49,3.94]  (11 tiny slivers, 0.039mm² total) |
| 4.2 | C++    | 1     | 661.334  | [-26.89,-9.69]-[16.43,9.69]  (single coherent region) |
| 4.2 | RUST   | 5     | 644.845  | [-26.06,-9.69]-[17.25,9.69]  (fragmented into 5) |

## stInternalSolid SurfaceFills entering detect_narrow
### pz=0.6  (THE DOMINANT DEFICIT — gcode layer 3, single worst layer)
- C++ : **1 expolygon, area 466.96 mm2, narrow=0 -> KEPT** (rectilinear internal solid, fills full region).
- RUST: **244 expolygons, total area only 193.98 mm2, ALL narrow=true -> ALL NARROW** (Concentric, then starved by FillConcentric no-boundary-seed).
  - Largest rust frags: j0=79.15mm² (bbox y in [2.65,7.64]), j3=112.58mm² (bbox y in [-7.63,2.09]); the OTHER 242 are sub-2mm² slivers, ~230 of them <0.001mm².

### pz=4.2  (representative body layer)
- C++ : 2 expolygons (21.76 + 13.77 mm2), holes 0/0, both narrow -> both NARROW.
- RUST: 2 expolygons (21.82 + 13.20 mm2). j0 matches. **j1 has holes=1 (a spurious hole) -> FLOAT** where C++ j1 has holes=0 -> NARROW.

## DIAGNOSIS — answers knot (c)
The reclassification (detect_narrow) is NOT the root. detect_narrow is a faithful 1:1 port and
behaves correctly given its inputs. The root is **the stInternalSolid fill_surface SHAPE that
ENTERS group_fills is already wrong**, in TWO distinct ways:

(A) pz=0.6 — AREA + FRAGMENTATION. Rust's bottom internal-solid region is 194mm² shattered into
    244 pieces; C++ has a single coherent 466.96mm² region. Rust is missing ~273mm² of
    stInternalSolid area AND has fragmented what remains so every piece reads narrow.
    This is candidate (i): the input region shape differs. detect_narrow then correctly narrow-routes
    rust's slivers (they ARE narrow) — but C++ never reaches that branch because its region is whole
    and wide (narrow=0, KEPT). So the FillConcentric boundary-seed fix (b) cannot help here: the
    region is the wrong shape/area upstream, AND C++ doesn't even use Concentric here (it's KEPT
    rectilinear). Fixing FillConcentric would fill rust's WRONG 194mm² fragmented region — not converge.

(B) pz=4.2 — SPURIOUS HOLE. Rust's 2nd ISI expolygon carries an extra hole that flips it from the
    NARROW (Concentric) branch to the FLOAT (FloatingVerticalShell) branch — explaining the
    floating-shell-over / ISI-under split on body layers. C++'s same region is hole-free.

## WHERE THE WRONG SHAPE IS BORN (next step, upstream of fill)
The stInternalSolid surfaces are produced by detect_surfaces_type + process_external_surfaces +
discover_vertical_shells (the "diff_int_holes" wave noted in 83d024b). The 467->194mm² area loss and
244-way fragmentation at pz=0.6, and the spurious hole at pz=4.2, are upstream surface-classification
divergences — NOT a fill-stage bug. This CONTRADICTS the handoff's "everything upstream proven
faithful": the entering ISI surface area is off by 2.4x at the worst layer.

## VERDICT
The coupled fill-stage fix (FillConcentric boundary-seed + reclass correction) CANNOT converge this
gap, because at the dominant layer C++ does not use Concentric at all (region is KEPT rectilinear) and
the rust region is the wrong area+shape. The faithful next step is to fix the UPSTREAM stInternalSolid
surface generation so the region entering group_fills matches C++ (single ~467mm² coherent region at
pz=0.6, hole-free 13.77mm² region at pz=4.2). Shipping (b)+(c) now would be a non-faithful patch over
a wrong input.
