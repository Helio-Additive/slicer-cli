# Run log

- 2026-10-28 20:28 pre-flight: on battery 63% (user asked to plug in), disk 491 GB free, CLT + Homebrew OK, GitHub reachable, keep-awake on
- 2026-10-28 20:29 Phase 0 started: local copy at ~/Projects/Snapmaker/slicer-cli-snapmaker, branch snapmaker-engine from origin/main 4f245f1, push URL disabled
- 2026-10-28 20:32 Snapmaker fork added as submodule references/SnapmakerOrca at v2.4.1 (bac6fd5)
- 2026-10-28 20:43 local deps built (CGAL 5.6.3, libnoise-orca) into deps-prefix/; build-orca configured; build started (-j8)
- 2026-10-28 20:44 build-orca failed at once: SDK rejects macOS 10.15 target (D-002); reconfigured with 11.0, rebuilding
- 2026-10-28 20:45 Phase 1 prep: CMake engine=snapmaker added (ENGINE_ORCA+ENGINE_SNAPMAKER), Clipper2 optional (fork lacks it), version header from common_func.hpp; build-snapmaker configures OK (D-003)
- 2026-10-28 20:50 build-orca OK in 5 min (0 failures). Note: D-002/D-003 timestamps corrected to real clock times
- 2026-10-28 20:50 Checkpoint 1 PASSED; Phase 1 started: full snapmaker build with -k 0 to collect every error
- 2026-10-28 21:05 Checkpoint 2 PASSED (97 places). Phase 2 starting with helpers
- 2026-10-28 21:29 Phase 2 done: helpers frontend+presets ported 6 files; full snapmaker build links (after eigen target fix); orca still builds. First snapmaker slice OK (cube20). Inbox merged into DECISIONS/PORTING/QUESTIONS. u1-tests set ready (work/u1-tests)
- 2026-10-28 21:31 Checkpoint 3 PASSED. Parity: 4 of 6 compared outputs identical to Snapmaker's own CLI. Note: Snapmaker CLI writes Sentry crash dumps to ~/Library/Application Support/Snapmaker_Orca/SentryData/pending despite --datadir (network blocked, so nothing sent); left in place, reported
- 2026-10-28 21:32 Phase 4 under way: helpers snap-fixes (defaults, names, version gate, tower position, mixed filament, 10 regressions) and package (Bambu build, local package) started; u1-tests asked for a single.3mf project case. Stopped the crash bisect (superseded).
- 2026-10-28 21:35 single_project (STL as project) IDENTICAL to Snapmaker CLI; bare STL differs by plate/placement defaults (D-007). Snapmaker CLI crash worked around (D-006).
- 2026-10-28 22:01 snap-fixes merged & verified by coordinator: both test scripts pass on snapmaker+orca; Orca U1 G-code byte-identical to baseline; U1 defaults now 0.20mm Standard + Generic PLA; 6 of 10 compared outputs identical to Snapmaker CLI (rest explained)
- 2026-10-28 22:16 Independent review done: Bambu/Orca unchanged (unifdef-verified); 1 confirmed crash (--arrange 1) + 5 smaller items → helper review-fixes; 4 decision items → QUESTIONS.md
- 2026-10-28 22:26 package helper done: Bambu built (3:44) and passes its 3 test scripts; 3-program package in pkg-local/ passes test-complete-slice-path (309/0/4), test-packaged-slice, smoke-clean-host; packaged slicer_cli-snapmaker four_colour IDENTICAL to Snapmaker CLI. Found temp-folder sweep bug (needs a 1-line change in all builds).
- 2026-10-28 22:29 review-fixes verified (arrange rc 0) & committed; D-160 temp-folder list; D-161 Snapmaker → CGAL 5.4; final verification run started
