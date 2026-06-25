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

## THE FIX (faithful, in progress)
Apply the same object/instance transformation to the mesh in rust BEFORE slicing (currently rust slices
the raw `load_stl` mesh with no trafo). This is the byte-faithful fix — it should:
  - move the floor to 0.299999237 (off-plane) → li=1 slices to 1 clean loop, 0 holes,
  - collapse the spurious li=2 BottomBridge (290→~13mm²), restore InternalSolid (→~494, coherent),
  - and also fix the +0.83mm X coordinate offset (bonus toward byte-parity).
NEXT: dump C++'s exact instance trafo matrix (Transform3d) + a floor vertex before/after, and replicate
the arithmetic (double-precision matrix multiply then cast to float) in the rust mesh-load path.
GUARDRAILS: the transform touches all 240 layers — verify per-feature (ISI→494, floating→170, bridge→237),
outer-wall G1 ~22087, time ~43m, build green.

## INSTRUMENTATION (env-gated; C++ probes to be reverted)
- rust: triangle_mesh_slicer.rs slice_facet_at_zs (F2FACET), app_slice.rs (F2FLOOR dz print).
- C++ : TriangleMeshSlicer.cpp slice_facet_at_zs (F2FACET + F2FLOOR).
