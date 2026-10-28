# The Snapmaker build (local)

`slicer_cli-snapmaker` is slicer-cli's front end compiled against **Snapmaker Orca v2.4.1**, Snapmaker's fork of OrcaSlicer (`references/SnapmakerOrca`, tag `v2.4.1`, `bac6fd5`). The goal is U1 G-code identical to what Snapmaker's own slicer produces.

This branch (`snapmaker-engine`) is local only. Its push URL is disabled.

## Build (macOS arm64)

```sh
decision-log/scripts/local-deps.sh          # CGAL 5.6.3 + Orca libnoise into deps-prefix/ (no sudo)
decision-log/scripts/local-deps-bambu.sh    # CGAL 5.4 (patched) + Bambu libnoise into deps-prefix/
decision-log/scripts/configure.sh snapmaker build-snapmaker
cmake --build build-snapmaker --target slicer_cli -j8
scripts/package-local-macos.sh pkg-local    # all three programs; validates an extracted copy
```

`configure.sh` builds for macOS 11.0. This machine's SDK refuses the release target of 10.15 (D-002).

## How the engine is wired

- **CMake.** `-DENGINE=snapmaker` defines both `ENGINE_ORCA` and `ENGINE_SNAPMAKER`, so every Orca code path applies. `ENGINE_SNAPMAKER` blocks mark where the fork differs (D-003).
- **Fork-specific build settings:**
  - the fork's bundled Eigen 3.3.7 (D-004);
  - CGAL 5.4, as the fork pins (D-161);
  - no Clipper2;
  - version macros from `src/common_func/common_func.hpp`.
- **Front-end differences.** The fork branched from upstream around v2.3.1, before upstream's 2.4 multi-extruder work (filament maps, extruder/nozzle variants, G-code check results, wipe tower types). Those steps are skipped or replaced with the fork's own logic. Every place is listed in `decision-log/PORTING-LOG.md`, and every choice in `decision-log/DECISIONS.md`.
- **Profiles and resources:**
  - in a checkout: `references/SnapmakerOrca/resources`;
  - in a package: `resources/profiles-snapmaker` and `resources/snapmaker/info`.

## What matches Snapmaker Orca

Compared with Snapmaker Orca 2.4.1's own command line, run headless with network blocked (`decision-log/scripts/snapmaker-cli.sh`), on the U1 test set in `work/u1-tests/`.

- **Identical G-code:** 4-colour, Graphic Effect Plate, high-flow, single-colour project, and plate 1 of the two-plate projects.
- **One coordinate differs by 0.001 mm:** plate 2 of the two-plate projects.
- **Different by decision:**
  - Mixed filament: we follow the desktop's path, so a wipe tower is placed. Snapmaker's CLI places none (D-141 and the snap-fixes entries).
  - Bare STL with no project: Snapmaker's CLI uses the Cool Plate and a different placement; we follow the desktop's U1 defaults (D-007).
- **Snapmaker's own CLI refuses its own 2.4.1 projects without `--allow-newer-file`; ours accepts them** (version gate, provisional).
- **Colour mixing:** 20 of 21 tests in the colour-mixing set are identical (D-163).
  - The set is `work/u1-tests/gen_mixing.py`, `run_mix_tests.sh` and `compare_mix.py`, run on Snapmaker-saved projects.
  - It covers the mixing modes, patterns, gradients, Local-Z, cadence, dithering, bias and offsets, painted mixed zones, and the Color Mixing process.
  - The one difference: a plate whose only object uses a mixed colour keeps its prime tower with us, and has none with Snapmaker's CLI (D-142, provisional).

## Known gaps

- No thumbnails (true of slicer-cli generally).
- `--downward-check` takes a value on this build (the fork defines it twice), and it repeated one printer name in one test.
- The U1 default process and filament, and how Snapmaker-saved projects are loaded, are provisional decisions. See `decision-log/QUESTIONS.md`.
- Linux and Windows are not built.

## Moving to a newer Snapmaker release

1. `git -C references/SnapmakerOrca fetch --depth 1 origin tag <tag> && git -C references/SnapmakerOrca checkout <tag>`
2. Rebuild `build-snapmaker`. Fix compile errors inside `ENGINE_SNAPMAKER` blocks, and log each one.
3. Re-run `work/u1-tests/run_u1_tests.sh` with both our program and the new Snapmaker app's CLI, then compare with `compare_gcode.py`.
