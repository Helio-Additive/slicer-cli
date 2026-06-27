# R76 — Residual sparse +55 localized to grid two-pass; BANKED at 1.017×. Next lever = the multilines+connect_infill emitter PAIR port.

Branch (diagnosis-only, no commits): `sparse-emitter2`, `grid-multilines` (both off parity @8f836e3).
Job: Benchy, `tests/configs/stl-inline-config.jsonnet`. Measure with `/tmp/feat_e2.py`.

## TL;DR

After R75 landed `overlap=0` (sparse +68→+55, total 1.030×→1.017×), the **residual
sparse +55** (native 531.63 / rust 586.64, +11.1% line length) was localized to the
**grid two-pass raster** and proven **entangled with `connect_infill`**. The faithful
fix is a two-part emitter PAIR port (`fill_surface_by_multilines` + the anchor-aware
`connect_infill`) — bigger than a bounded knob — so the lever was **BANKED at 1.017×**.
No code shipped this round (diagnosis only); default unchanged.

## Per-candidate localization (each ruled with one both-engine number)

Re-measured on the post-overlap baseline (overlap=0 default), sparse grid geometry:
rust **16555mm / native 14907mm = +11.1%** (~1648mm). Signature: rust 2423
extrude-moves @ 6.83mm avg + 744 travels + 335 runs vs native 2608 @ 5.72mm + 316
travels + 226 runs. Per-layer uniform +12–18% (native 2 runs/layer, rust 3–4).

- **CONNECTORS / connect_infill — RULED OUT (as length source).** `SPARSE_NOCONNECT`
  A/B (skip the connect pass for sparse): sparse 586.64→586.44 (−0.2), extrude_len
  16555→16543. Connecting adds ~0 length. (rust's `connect_infill` IS a simplified
  stub — straight-line `2.5*spacing` rule, ignores anchor caps — but it only re-chains
  existing lines.)
- **ANCHORING — RULED OUT.** Config faithful: `region_config.infill_anchor` = 400% /
  `infill_anchor_max` = 20mm (region_config.rs:120, loaded from `sparse_infill_anchor`),
  resolves to anchor = 4·spacing, anchor_max = 20 — matching C++ Fill.cpp:287-292. The
  anchor caps live inside C++ `connect_infill` (which isn't adding the length).
- **SPACING / PITCH — RULED OUT.** Realized diagonal pitch ~3mm both; `INFILL_OVERLAP
  _OVER_SPACING` = 0.45 both; `aoffset1/aoffset2` formula matches now that overlap=0.
- **RASTER + GRID TWO-DIRECTION — THE ROOT.** C++ `FillGrid::fill_surface` =
  `fill_surface_by_multilines` (FillRectilinear.cpp:3032-3120): builds BOTH sweep
  directions into ONE `fill_lines` set over a SHARED `poly_with_offset_base`
  (`make_fill_lines`), then `connect_infill` ONCE on the combined set + a layer-parity
  reverse. Rust `generate_fill_rectilinear` (fill_rectilinear.rs:2214-2266) runs
  `fill_surface_by_lines` TWICE (each pass independently offsets/clips/graph-connects/
  traverses), then concatenates → more total raster line.

## The decisive A/B (why it's a PAIR, not a bounded swap)

`GRID_COMBINED=1` A/B (both sweeps' raw segments via `dont_connect`, single combined
connect, vs rust's two self-connected passes):

| FEATURE | baseline (overlap=0) | GRID_COMBINED |
|---|---|---|
| Sparse infill | +55.01 | **−113.73** (417.9 vs native 531.6) |
| TOTAL | +67.20 | −111.93 |

geom: native 14907mm / 226 runs / 316 travels → combined-rust 12598mm / **1041 runs /
1671 travels**.

**Combining the two passes moves sparse massively (586→417) — so the two-pass IS the
mechanism.** BUT the minimal combine OVER-corrects to −113 UNDER native and fragments
the topology (1041 runs / 1671 travels vs native's 226 / 316), because `dont_connect`
strips the LEGITIMATE within-direction chaining. The combined sweep's material outcome
is DETERMINED by which connect runs on it:
- rust's simplified connect on combined raster → −113, fragmented;
- native's anchor-aware `connect_infill` on combined raster → 531, chained.

So the combined raster and the faithful connect are **NOT separable**, and the result
**cannot be validated incrementally** (combined sweep alone over-corrects; only the
pair lands near 531).

## NEXT-SESSION LEVER (documented, scoped)

Port the EMITTER PAIR together as one lever:
1. **`fill_surface_by_multilines`** (FillRectilinear.cpp:3032-3071): the `SweepParams`
   loop emitting raw segments (`make_fill_lines`) in a SHARED unrotated frame over a
   shared `poly_with_offset_base`, replacing rust's two-pass concat for the GRID case.
   Keep single-direction RECTILINEAR (solid/top/bottom) on its existing path.
2. **Faithful `connect_infill`** (FillBase.cpp:1501-1660, ~160 lines): the anchor-aware
   `BoundaryInfillGraph` + `take_limited` + `anchor_length` / `anchor_length_max` caps,
   run ONCE on the combined set, + the layer-parity reverse.

Material upside ~+55 (sparse → native 531). This ALSO retires the `connect_infill` stub
(crates/libslic3r-rs/src/fill/mod.rs:1454 — a broad chaining/travel/seam debt affecting
all infill, not just grid). Risk: the rectilinear emitter is shared-ish — verify
SOLID/TOP/BOTTOM stay byte-identical (single-direction path untouched) and ISI/floating
don't regress.

## State (banked)

Default UNCHANGED at the overlap=0 state (parity @8f836e3): ISI −30.18 / floating
+31.56 / sparse +55.01 / bridge +2.96 / **total +67.41 (1.017×)**, walls + gap at
parity, time 44m23s. Both diagnosis branches have 0 commits; all instrumentation
stripped; C++ references reverted; build green.

## Run ledger (this multi-day run, all faithful, all landed unless noted)

R65 slicer f32 center round-trip; R67/R68 Arachne SkeletalTrapezoidation graph builder
(keystone); R69 faithful FillFloatingConcentric; R72/R73 Clipper2-Z shim + faithful
wave_seeds (gated on `wave-seeds`, material-neutral — fragment count is not the material
lever, R74); R74 group_fills post-loop + bridges-first Ord (bridge −6.75); R75
`overlap=0` (sparse −13, total 1.030×→1.017×, the best yet). R76 (this) banks the
residual sparse +55 with the emitter-pair port scoped as the next lever.
