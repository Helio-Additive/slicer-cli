# R65 — ROOT CAUSE: rust slices the RAW mesh; C++ applies an instance transform that nudges the floor off the slice plane

Branch: f2-slicefix (off the parity branch). Both engines instrumented (F2FACET/F2FLOOR/dz, env-gated).

## THE FACTS (both-engine, ground-truthed against the STL bytes)
- The Benchy cabin floor is a flat horizontal plate: **10742 STL vertices at exactly z=0.300000012
  (0x3e99999a = f32(0.3))** — confirmed by reading `_downloads/3DBenchy.stl` raw bytes.
- The li=1 slice plane is `slice_z = 0.5*(0.2+0.4) = 0.3 = f32(0.3) = 0.300000012` — IDENTICAL to the
  floor's stored z.
- **RUST** feeds the raw loaded mesh to the slicer → floor vertices stay at **0.300000012 == slice_z**.
  The 1780 floor facets are perfectly horizontal (min_z==max_z) sitting EXACTLY on the plane → the
  degenerate coincident-plane case → the on-plane vertex handling emits 8 spurious interior hole-loops.
- **C++** feeds an instance-transformed mesh → the same floor vertices arrive at **0.299999237**
  (~7.75e-7 = ~21 ULPs BELOW the plane) → floor is off-plane → sliced cleanly → 1 solid loop, no holes.
- RUST bed-drop `dz = 0` (bbox.min.z = -0.0) — the app_slice.rs:68 drop is a NO-OP, NOT the cause.

## CONCLUSION
The cascade root (R63/R64: rust's li=1 slice has 8 phantom holes) is because **rust slices the mesh in
its RAW STL coordinates, while C++ slices the mesh AFTER applying the object/instance transformation**
(main.cpp ~948-1003 applies instance transforms + a plate-local translation; PrintObject then slices the
transformed mesh). That transform perturbs the floor z by ~7.75e-7, moving it off the exactly-coincident
slice plane. It ALSO explains the long-standing **+0.83mm X-translate** between the two engines' contours
(R64) — same missing transform.

The STL value (0.300000012) happens to equal f32(0.3) exactly, and the slice grid puts a plane at exactly
0.3 (layer midpoint li=1) — so without C++'s perturbing transform, rust lands on the knife-edge.

## THE EXACT MECHANISM (F2TRAFO dump confirmed)
The slicing transform C++ applies (PrintObjectSlice.cpp:60, `trafo_centered() * volume.get_matrix()`) is
essentially identity + a Z translate of EXACTLY +24 (matrix `[0 0 1 24]`; X translate negligible 8.4e-8).
The slice planes are object-frame `zs` (zs[1]=f32(0.3)=0.300000012). C++ STORES the ModelVolume mesh as
**f32 centered** on its bbox (center_z=24): the floor STL value f32(0.3)=0.300000012 becomes
`f32(0.3 - 24) = f32(-23.7) = -23.700000763` (f32 is ~21 ULPs coarser near 24), then the +24 instance
trafo gives `-23.700000763 + 24 = 0.299999237`. That is 7.75e-7 BELOW slice plane 0.300000012 → off-plane
→ clean floor. Rust stores vertices in **f64** and only casts to f32 at slice time, so it keeps the exact
f32(0.3) and sits bit-coincident with the plane. Verified: `f32(f32(0.3-24)+24) = 0.299999237`.

## THE FIX (LANDED) — `TriangleMesh::quantize_f32_center_roundtrip()`
Bake C++'s f32 center-store/instance-place round trip into the f64 mesh vertices before slicing
(app_slice.rs, after load): per vertex `v = ((v as f32 - center as f32) + center as f32) as f64` on all
3 axes, center = mesh bbox center. Net-zero for geometry not on a slice plane; reproduces C++'s f32
quantization for geometry that is. (An f64 round trip is a no-op — must be f32; our first attempt missed
this and had zero effect.)

## RESULT (clean A/B, NO_F32RT gate; main-session verified)
- li=1 raw loops 10 → **1** (matches C++); F2FACET cap-facets at z=0.3: 1780 → **0** (floor off-plane).
- Layer-3 cabin-floor ISI: rust 13.07 → **38.19** (native 38.08) — **at parity** (was the −25 worst layer).
- Per-feature ALL moved toward native: Outer-wall G1 +212→**+158**, Inner-wall G1 +578→**+554**,
  Outer-wall mat 1010.5→**1005.2** (nat 1003.1), Inner-wall mat 1003.1→**998.0** (nat 995.2),
  ISI mat 414.1→**418.1**. Time 42m49s, gap-fill parity, build green. NO guardrail regression
  (outer-wall G1 IMPROVED — opposite of the rejected R59 slicer-fix at +371).
- Aggregate total 3850.1→3846.6: dropped only because the over-extruded walls correctly came down; the
  remaining ISI deficit (−76) is the SEPARATE distributed FillConcentric starvation (body+top layers),
  not the floor. This fix is the slicer half; FillConcentric is the next lever.

## INSTRUMENTATION (all removed/reverted in the final commit)
- rust: triangle_mesh_slicer.rs (F2FACET), app_slice.rs (F2FLOOR) — removed.
- C++ : TriangleMeshSlicer.cpp (F2FACET/F2FLOOR), PrintObjectSlice.cpp (F2TRAFO) — reverted + rebuilt clean.
