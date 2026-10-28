# Rules for every helper (read first)

Work folder: `~/Projects/Snapmaker/slicer-cli-snapmaker` (a local copy of Helio's slicer-cli; branch `snapmaker-engine`).
Goal of the whole job: a third program built with `-DENGINE=snapmaker` against Snapmaker's OrcaSlicer fork (`references/SnapmakerOrca`, tag v2.4.1) whose U1 G-code matches Snapmaker Orca's desktop app.

## Background you need
- The front end already supports two engines through `#ifdef ENGINE_BAMBU` / `#ifdef ENGINE_ORCA`.
- The Snapmaker build defines **both** `ENGINE_ORCA` and `ENGINE_SNAPMAKER`, so every Orca block is compiled for it.
- The Orca blocks were written against upstream OrcaSlicer **v2.4.2** (`references/OrcaSlicer`). The Snapmaker fork branched from upstream around **v2.3.1** (Oct 2025), so it lacks most of upstream's 2.4-era multi-extruder APIs. That is what the compile errors are: filament maps, extruder/nozzle variants, gcode check results, wipe tower type, and a few CLI error codes.

## Hard rules
1. **Edit only the files you own** (listed in your brief). Never edit `references/`, `CMakeLists.txt`, or another helper's files. If you need one of those changed, write the request in your inbox file and carry on.
2. **The Bambu and Orca programs must compile exactly as before.**
   - Put every change inside a Snapmaker-only branch: `#ifdef ENGINE_SNAPMAKER … #else <original code unchanged> #endif`, or `#ifndef ENGINE_SNAPMAKER` around code the Snapmaker build must skip.
   - Don't reformat or touch unrelated code.
3. **Follow Snapmaker's engine; don't recreate upstream on top of it.**
   - Before deciding, look at how Snapmaker's own code does the same job: `references/SnapmakerOrca/src/Snapmaker_Orca.cpp` (its CLI) and `references/SnapmakerOrca/src/libslic3r/`. Mirror that.
   - If a 2.4 feature simply doesn't exist in the fork (for example filament maps), skip that step on the Snapmaker build. Leave a result field out, or report it the way Snapmaker's CLI would.
   - Where the user would notice (a flag, or a `result.json`/event field), say so plainly. For example, emit the existing diagnostic/refusal mechanism with "not supported on the Snapmaker build".
   - **Never invent data** to fill a field the engine doesn't produce.
4. **Never edit engine files, and no override-layer files.** If you believe one is needed, explain why in your inbox and stop that item.
5. **No git commands that change anything** (no commit, stash, checkout, reset). `git diff` and `git status` are fine. **Never push.**
6. **Compile only through the lock script.** It compiles just your files and waits for its turn:
   - `decision-log/scripts/compile.sh snapmaker <your files…>`
   - `decision-log/scripts/compile.sh orca <your files…>` (proves the Orca build is unaffected)

   Never run a full build, cmake, or a bare ninja. Never delete build folders.
7. Done means **your files compile with zero errors in both `snapmaker` and `orca`**. Fix new warnings you introduce. Linking is the coordinator's job.

## Logging (required)
- **Decisions:** append to `decision-log/inbox/<your-name>.md`, one entry per real choice:
  ```
  ### <short title>
  Time: <run `date '+%Y-%m-%d %H:%M'`; never guess>
  What:    …
  Why:     …
  Options: (a) … (b) …
  Chosen:  … because …
  Evidence: file:line in our code and in references/SnapmakerOrca (or OrcaSlicer)
  User-visible effect: none / <what the user sees>
  Status: final | provisional (the user should review)
  ```
  A choice is **provisional** if it changes what a user sees or gets. Mechanical renames are final.
- **Porting rows:** append to `decision-log/inbox/<your-name>-porting.md`, one table row per place you changed:
  `| file:line | what broke | what we did | decision title |`
- **Questions** only the user can answer: append to `decision-log/inbox/<your-name>-questions.md`, with the provisional answer you used.

## Your final report
Keep it short:
- each file you changed, with the number of places;
- the result of the last `compile.sh` run for snapmaker and orca (paste the last lines);
- a list of the provisional decisions;
- anything you could not resolve.

Say exactly what you verified and what you did not.
