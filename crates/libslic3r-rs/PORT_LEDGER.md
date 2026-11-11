# Port ledger

The per-unit counts that used to stand here (164 of 278 units "done", last regenerated 2026-06-11) were
hand-kept statuses and are out of date. What is ported, file by file and function by function, is now
measured from the two source trees: [`docs/rust-engine/FILE_MAP.md`](../../docs/rust-engine/FILE_MAP.md),
written by `python3 scripts/parity/file_map.py`.

`PORT_LEDGER.json` stays as the place to mark a unit `partial` with what blocks it (AGENTS.md, "The Rust
engine"); its `status` values are not a measure of fidelity.

## Headline of the file map

| | |
|-|-|
| C++ units | 310 |
| with a Rust file of the same name | 277 (89%) |
| of those, same words | 276 |
| of those, same letters but other word breaks | 1 |
| of those, in another directory than the C++ file | 6 |
| C++ units with no Rust file | 33 |
| Rust files with no C++ file of that name | 48 (plus 21 `mod.rs`/`lib.rs`/`main.rs`) |
| Rust files that share a C++ unit's name with another Rust file | 7 |
| C++ functions (named, with a body) | 9052 |
| matched to a Rust function in the paired file | 3706 (41%) |
| estimate: faithful | 3102 (84% of matched) |
| estimate: partial | 374 (10% of matched) |
| estimate: divergent | 199 (5% of matched) |
| estimate: stub | 31 (1% of matched) |
| missing in the paired Rust file | 4678 |
| of those, a function of that name exists in another Rust file | 1612 |
| of the rest, another overload of the name is ported | 366 |
| of the rest, under 25 tokens (accessors, one-liners) | 762 |
| of the rest, 25 tokens or more | 1938 |
| in a C++ unit with no Rust file | 668 |
| Rust functions with no C++ function of that name in the pair | 4505 |

By tier:

| Tier | C++ units | paired | C++ functions | matched | faithful | partial | divergent | stub | missing |
|-|-|-|-|-|-|-|-|-|-|
| 1 | 180 | 173 | 6851 | 2804 | 2354 | 277 | 153 | 20 | 4047 |
| 2 | 15 | 15 | 392 | 150 | 124 | 20 | 5 | 1 | 242 |
| 3 | 92 | 65 | 1381 | 465 | 389 | 41 | 30 | 5 | 916 |
| 4 | 23 | 23 | 435 | 243 | 217 | 19 | 2 | 5 | 192 |
