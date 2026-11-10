# Rounds from R809: against this repository's `slicer_cli`

From R809 the oracle is this repository's `slicer_cli` (Bambu Studio
02.09.01.70, `references/BambuStudio` at `56a57d0`), not the 02.06.00.51 engine
the earlier rounds were scored against. Scores are in-order matched lines from
`scripts/parity/seq_parity.py`, on the job configs in
`tests/rust-engine/configs/` (`benchy-016`, `benchy-016-arachne`, `nu3mf` for
Majora), with both programs built on the same machine against the same Eigen
(5.0.1) and Boost (1.90) headers.

## Baseline

Taken before R809, with the STL jobs resolving their profiles from
`references/BambuStudio` (02.09.01.70 profiles):

| Fixture | In order | Content |
|-|-|-|
| Benchy classic | 30,501 (15.72%) | 41,420 (21.35%) |
| Benchy arachne | 32,785 (13.69%) | 55,455 (23.15%) |
| Majora | 1,513,431 (55.92%) | 1,745,602 (64.50%) |
| Majora, prime tower | 83.5% | |

## Rounds

| Round | What | Gate | Benchy classic | Benchy arachne | Majora |
|-|-|-|-|-|-|
| R809 | STL placed on the plate as `slicer_cli` does (`desktop_place_on_bed`, cli_model_load.cpp 280-378): mesh stored minus its bounding-box centre in f32, volume offset re-centred twice in f64, instance at the bed centre with a truncating `new_scale` shift (160 mm scales to 15999999), sliced through the exact composed f32 transform | `STL_DESKTOP_PLACE` (on) | 30,501 → 50,372 (25.94%) | 32,785 → 67,721 (28.29%) | unchanged (3MF path) |

## Findings

- R809: after placement, straight-wall coordinates agree to the last digit, so
  the frame is exact. What still differs on Benchy is where long wall lines are
  split for per-segment speed, the seam start, and feed rates; the point sets
  agree least on the first ten layers.
