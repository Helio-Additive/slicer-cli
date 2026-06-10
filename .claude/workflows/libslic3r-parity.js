export const meta = {
  name: 'libslic3r-parity',
  description: 'Drive crates/libslic3r-rs toward exact 1:1 parity with C++ libslic3r (BambuStudio). MAP=gap+native-dep audit, PORT=faithful translate+review+build, VERIFY=gcode parity.',
  whenToUse: 'Run with args {phase:"map"} to (re)build the C++<->Rust correspondence, gap report, and wasm/native-dependency audit. args {phase:"port", items:[cpp paths]} to faithfully port a bounded batch. args {phase:"verify"} for gcode parity.',
  phases: [
    { title: 'Inventory', detail: 'one agent: bash-enumerate C++ + Rust files, signals, chunk plan' },
    { title: 'Map', detail: 'one agent per chunk: classify status, structural fidelity, native deps' },
    { title: 'Audit', detail: 'one agent: cluster native deps -> minimal vendored crates + wasm review' },
    { title: 'Critique', detail: 'one agent: completeness check, unmapped files, inconsistencies' },
  ],
}

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------
const REF = 'libslic3r/bambustudio/references/BambuStudio/src/libslic3r'
const CRATE = 'crates/libslic3r-rs'
const SRC = `${CRATE}/src`
const phaseArg = (args && args.phase) ? args.phase : 'map'

// ----------------------------------------------------------------------------
// Schemas
// ----------------------------------------------------------------------------
const INVENTORY_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['chunks', 'rustOrphans', 'totals'],
  properties: {
    chunks: {
      type: 'array',
      items: {
        type: 'object',
        additionalProperties: false,
        required: ['id', 'subdir', 'files'],
        properties: {
          id: { type: 'string' },
          subdir: { type: 'string', description: 'repo-root-relative subdir under the ref, or "(root)"' },
          files: {
            type: 'array',
            items: {
              type: 'object',
              additionalProperties: false,
              required: ['cpp', 'loc', 'nativeIncludes', 'candidateRust', 'rustLoc', 'rustTodoMarkers'],
              properties: {
                cpp: { type: 'string', description: 'repo-root-relative path to .cpp/.hpp/.h' },
                loc: { type: 'integer' },
                nativeIncludes: { type: 'array', items: { type: 'string' }, description: 'detected native libs (CGAL, boost, Eigen, tbb, libigl/igl, clipper, admesh, qhull, nlopt, opencv, miniz, expat, ...)' },
                candidateRust: { type: ['string', 'null'], description: 'repo-root-relative best-guess Rust counterpart via PascalCase->snake_case, or null' },
                rustLoc: { type: ['integer', 'null'] },
                rustTodoMarkers: { type: ['integer', 'null'], description: 'count of unimplemented!/todo!/TODO/panic!("not implemented") in the candidate Rust file' },
              },
            },
          },
        },
      },
    },
    rustOrphans: { type: 'array', items: { type: 'string' }, description: 'Rust src files with no obvious C++ counterpart' },
    totals: {
      type: 'object',
      additionalProperties: false,
      required: ['cppFiles', 'cppLoc', 'rustFiles', 'rustLoc'],
      properties: {
        cppFiles: { type: 'integer' },
        cppLoc: { type: 'integer' },
        rustFiles: { type: 'integer' },
        rustLoc: { type: 'integer' },
      },
    },
  },
}

const MAP_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['records'],
  properties: {
    records: {
      type: 'array',
      items: {
        type: 'object',
        additionalProperties: false,
        required: ['cpp', 'rust', 'status', 'structuralFidelity', 'confidence', 'cppLoc', 'rustLoc', 'keySymbols', 'nativeDeps', 'priority', 'effort', 'blockers', 'notes'],
        properties: {
          cpp: { type: 'string' },
          rust: { type: ['string', 'null'], description: 'confirmed Rust counterpart (corrects the candidate), or null if none exists' },
          status: { type: 'string', enum: ['missing', 'stub', 'partial', 'done'], description: 'missing=no Rust; stub=exists but empty/unimplemented; partial=exists but diverges/incomplete; done=faithful full port' },
          structuralFidelity: { type: 'string', enum: ['mirrors', 'diverges', 'na'], description: 'does the Rust mirror the C++ file/function/class layout' },
          confidence: { type: 'number', description: '0..1 confidence in this classification' },
          cppLoc: { type: 'integer' },
          rustLoc: { type: 'integer' },
          keySymbols: {
            type: 'object',
            additionalProperties: false,
            required: ['total', 'ported', 'missingList'],
            properties: {
              total: { type: 'integer', description: 'count of public functions/classes/structs in the C++ file' },
              ported: { type: 'integer' },
              missingList: { type: 'array', items: { type: 'string' }, description: 'names of notable symbols not yet ported (cap ~15)' },
            },
          },
          nativeDeps: {
            type: 'array',
            items: {
              type: 'object',
              additionalProperties: false,
              required: ['lib', 'functions', 'wasmSafe'],
              properties: {
                lib: { type: 'string' },
                functions: { type: 'array', items: { type: 'string' }, description: 'specific symbols/functions actually used from the lib' },
                wasmSafe: { type: 'boolean', description: 'false if it dynamically links / needs system/native code unavailable under wasm32' },
              },
            },
          },
          priority: { type: 'string', enum: ['critical', 'high', 'medium', 'low'], description: 'criticality to the FFF slicing pipeline that produces gcode' },
          effort: { type: 'string', enum: ['S', 'M', 'L', 'XL'] },
          blockers: { type: 'array', items: { type: 'string' }, description: 'other files/crates this depends on before it can be ported faithfully' },
          notes: { type: 'string' },
        },
      },
    },
  },
}

const AUDIT_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['libraries', 'currentDepsWasmReview'],
  properties: {
    libraries: {
      type: 'array',
      items: {
        type: 'object',
        additionalProperties: false,
        required: ['name', 'wasmCompatible', 'usedByCount', 'usedBySample', 'functionsUsed', 'currentRustCrate', 'currentCrateWasmSafe', 'proposedCrateName', 'recommendation'],
        properties: {
          name: { type: 'string' },
          wasmCompatible: { type: 'boolean', description: 'is the C++ usage achievable under wasm32 without system/dynamic links' },
          usedByCount: { type: 'integer' },
          usedBySample: { type: 'array', items: { type: 'string' }, description: 'sample of cpp files using it (cap ~10)' },
          functionsUsed: { type: 'array', items: { type: 'string' }, description: 'the EXACT minimal surface actually used (so a micro-crate can replicate only these)' },
          currentRustCrate: { type: ['string', 'null'], description: 'existing Cargo dep already covering this (e.g. geo-clipper, clipper2, boostvoronoi) or null' },
          currentCrateWasmSafe: { type: ['boolean', 'null'] },
          proposedCrateName: { type: 'string', description: 'name for a minimal vendored crate replicating ONLY functionsUsed' },
          recommendation: { type: 'string' },
        },
      },
    },
    currentDepsWasmReview: {
      type: 'array',
      items: {
        type: 'object',
        additionalProperties: false,
        required: ['crate', 'wasmSafe', 'reason'],
        properties: {
          crate: { type: 'string' },
          wasmSafe: { type: 'boolean' },
          reason: { type: 'string' },
        },
      },
    },
  },
}

const CRITIC_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['unmappedCpp', 'unexplainedRustOrphans', 'auditGaps', 'inconsistencies', 'overallCompleteness', 'verdict'],
  properties: {
    unmappedCpp: { type: 'array', items: { type: 'string' }, description: 'C++ files present in the ref but absent from the map records' },
    unexplainedRustOrphans: { type: 'array', items: { type: 'string' } },
    auditGaps: { type: 'array', items: { type: 'string' }, description: 'native libs referenced in records but missing from the audit' },
    inconsistencies: { type: 'array', items: { type: 'string' } },
    overallCompleteness: { type: 'number', description: '0..1 estimate of how complete the existing Rust port is, weighted by priority' },
    verdict: { type: 'string' },
  },
}

const PORT_SCHEMA = {
  type: 'object', additionalProperties: false,
  required: ['cpp', 'rust', 'changed', 'summary', 'symbolsPorted', 'remaining'],
  properties: {
    cpp: { type: 'string' }, rust: { type: 'string' },
    changed: { type: 'boolean' }, summary: { type: 'string' },
    symbolsPorted: { type: 'array', items: { type: 'string' } },
    remaining: { type: 'array', items: { type: 'string' } },
  },
}
const REVIEW_SCHEMA = {
  type: 'object', additionalProperties: false,
  required: ['faithful', 'fidelityScore', 'divergences', 'verdict'],
  properties: {
    faithful: { type: 'boolean' },
    fidelityScore: { type: 'number' },
    divergences: { type: 'array', items: { type: 'object', additionalProperties: false, required: ['cppRef', 'issue', 'severity'], properties: { cppRef: { type: 'string' }, issue: { type: 'string' }, severity: { type: 'string', enum: ['blocker', 'major', 'minor'] } } } },
    verdict: { type: 'string' },
  },
}
const BUILD_SCHEMA = {
  type: 'object', additionalProperties: false,
  required: ['compiles', 'errors', 'log'],
  properties: { compiles: { type: 'boolean' }, errors: { type: 'array', items: { type: 'string' } }, log: { type: 'string' } },
}

// ============================================================================
// PHASE: MAP
// ============================================================================
async function runMap() {
  phase('Inventory')
  log('Enumerating C++ ref + Rust crate, computing signals, building chunk plan…')
  const inv = await agent(
`You are the INVENTORY agent for a C++ -> Rust faithful-port parity audit.

C++ reference root (repo-root-relative): ${REF}
Rust crate src root: ${SRC}

Do ALL of this with Bash (find/wc/grep/sed) — be exhaustive, this is read-only:

1. Enumerate EVERY C++ source/header under ${REF}: files matching *.cpp *.hpp *.h *.c *.cc *.cxx *.ipp. Record repo-root-relative path + line count (wc -l).
2. Enumerate EVERY *.rs under ${SRC}: path + line count.
3. For each C++ file, detect native-library includes by grepping its #include lines. Map to canonical lib names: CGAL, boost, Eigen, tbb, libigl (igl/), clipper (clipper/ClipperLib/polyclipping), admesh, qhull, nlopt, opencv, miniz, expat, zlib, nanosvg, earcut, libnest2d, openvdb, tbb. List the libs each file pulls in.
4. For each C++ file compute its best-guess Rust counterpart by the project's naming convention: PascalCase basename -> snake_case with digits split (examples observed: AABBMesh.cpp->aabb_mesh.rs, MultiPoint.cpp->multi_point.rs, QuadricEdgeCollapse.cpp->quadric_edge_collapse.rs, PrincipalComponents2D->principal_components2_d.rs, SurfaceCollection->surface_collection.rs). Honor subdir nesting (Fill/ -> fill/, GCode/ -> gcode/, Support/ -> support/, Arachne/ -> arachne/, etc.). Verify the candidate file actually exists under ${SRC}; if not, set candidateRust=null. Record candidate path + its line count + count of TODO markers (grep -cE 'unimplemented!|todo!|TODO|unreachable!|panic!\\("not' on the candidate; null if no candidate).
5. Identify Rust .rs files that have NO plausible C++ counterpart (orphans) -> rustOrphans.
6. Group the C++ files into BALANCED chunks of ~18-22 files each for downstream parallel mapping. Keep a subdir's files together; if a subdir (or the ref root "(root)") exceeds ~22 files, split it alphabetically into id suffixes like "Fill-a","Fill-b","(root)-a". Give every chunk a stable id and the subdir it covers.
7. Report totals.

.cpp and its matching .hpp/.h count as the SAME logical file's two halves — pair them: prefer listing the .cpp as the primary 'cpp' field, but if only a header exists list the header. Do not double count a class across .cpp+.hpp; treat the pair as one entry keyed on the .cpp when present (note the header in the path is fine, but avoid emitting both as separate entries when they are the same class). Headers with no .cpp (header-only) are their own entries.

Return the structured inventory. Paths MUST be repo-root-relative.`,
    { schema: INVENTORY_SCHEMA, phase: 'Inventory', label: 'inventory' }
  )

  if (!inv || !inv.chunks || !inv.chunks.length) throw new Error('Inventory produced no chunks')
  log(`Inventory: ${inv.totals.cppFiles} C++ files (${inv.totals.cppLoc} LOC), ${inv.totals.rustFiles} Rust files (${inv.totals.rustLoc} LOC), ${inv.chunks.length} chunks, ${inv.rustOrphans.length} orphans.`)

  phase('Map')
  const mapResults = await parallel(inv.chunks.map((chunk) => () =>
    agent(
`You are a MAP agent classifying C++ -> Rust port fidelity for one chunk. This is read-only; do NOT edit anything.

C++ ref root: ${REF}    Rust crate src: ${SRC}

Your chunk (id="${chunk.id}", subdir="${chunk.subdir}") with precomputed signals:
${JSON.stringify(chunk, null, 1)}

For EACH file in the chunk:
- Open the C++ file (Read; for big files read the head + skim with Grep for the symbol list). Identify its public symbols (classes/structs/free functions/enums) — count them (keySymbols.total).
- Open the candidate Rust file if any (correct the path if the candidate is wrong — search ${SRC} with Grep/Glob for the type name; set rust to the real path or null).
- Judge status:
    missing  = no Rust counterpart exists.
    stub     = Rust file exists but is essentially empty / unimplemented!/todo! / placeholder.
    partial  = Rust exists and implements some but not all symbols, OR diverges from C++ behavior/structure in ways that would break gcode parity.
    done     = every public symbol present and the implementation faithfully mirrors the C++ logic.
- Judge structuralFidelity: does the Rust mirror the C++ file/function/class organization (the project's explicit goal — translate "as close as possible to how the C++ files are")? mirrors | diverges | na(=missing).
- keySymbols.ported = how many of the C++ public symbols have a real Rust implementation. missingList = notable un-ported symbol names (cap 15).
- nativeDeps: from the C++ #includes, list each native lib and the SPECIFIC functions/types actually used from it (so a later micro-crate can replicate only those). wasmSafe=false if it needs dynamic/system linkage unavailable on wasm32.
- priority: criticality to the FFF slice->gcode pipeline (model load, slicing, perimeters/Arachne, infill/Fill, support, GCode export = critical/high; SLA, calibration, GUI-adjacent, formats other than 3mf/stl = lower).
- effort: S(<150 LOC) M(150-600) L(600-1500) XL(>1500), relative to the C++ LOC and dependency depth.
- blockers: other files/types this needs first.
- notes: one line — the single most important fact for whoever ports it.

Be accurate over optimistic: when unsure between partial and done, choose partial and lower confidence. Return one record per chunk file.`,
      { schema: MAP_SCHEMA, phase: 'Map', label: `map:${chunk.id}` }
    )
  ))

  const records = mapResults.filter(Boolean).flatMap((r) => r.records || [])
  log(`Mapped ${records.length} files.`)

  // ---- Aggregate (plain JS, no agent) ----
  const by = (key) => records.reduce((m, r) => { const k = r[key]; m[k] = (m[k] || 0) + 1; return m }, {})
  const statusCounts = by('status')
  const fidelityCounts = by('structuralFidelity')
  const priorityCounts = by('priority')
  const nativeFiles = records.filter((r) => r.nativeDeps && r.nativeDeps.length)
  const nativeLibSet = Array.from(new Set(nativeFiles.flatMap((r) => r.nativeDeps.map((d) => d.lib)))).sort()

  phase('Audit')
  log(`Auditing native dependencies across ${nativeFiles.length} files; libs: ${nativeLibSet.join(', ') || '(none detected)'}`)
  const audit = await agent(
`You are the NATIVE-DEPENDENCY / WASM audit agent. Goal: make crates/libslic3r-rs ship as a drop-in library that ALSO runs on wasm32 in the browser — meaning NO dynamic links to system packages/dylibs, and any upstream native functionality replicated by minimal vendored Rust crates scoped to ONLY the functions actually used.

Inputs:
- Native libs detected across the C++ ref: ${JSON.stringify(nativeLibSet)}
- Files using native libs (lib -> the specific functions the mappers saw used):
${JSON.stringify(nativeFiles.map((r) => ({ cpp: r.cpp, rust: r.rust, nativeDeps: r.nativeDeps })), null, 1)}

Also inspect the CURRENT Rust crate dependencies for wasm compatibility:
- Read ${CRATE}/Cargo.toml and the workspace ${'Cargo.toml'} / Cargo.lock as needed.
- Several existing deps wrap C/C++ via cc/build scripts and will NOT compile to wasm32-unknown-unknown (suspect: geo-clipper, clipper2, boostvoronoi, anything with a *-sys crate, zip/flate2 with system zlib, openssl). For EACH such current dep determine wasmSafe + reason, and whether a pure-Rust replacement exists.

For EACH native library actually used by the C++ code:
- Determine the EXACT minimal surface used (functionsUsed) — e.g. if only 2-3 CGAL functions are used (mesh boolean? convex hull? AABB tree?), name them. The whole point is to replicate ONLY those.
- wasmCompatible: can that surface be done in pure Rust / no system linkage.
- currentRustCrate: any existing Cargo dep already covering it, and currentCrateWasmSafe.
- proposedCrateName + recommendation: propose a minimal vendored crate boundary (pure-Rust, no_std-friendly where possible) replicating only functionsUsed, OR name an existing pure-Rust crate to adopt.

Use Bash grep across ${REF} to confirm/expand which symbols of each lib are referenced (e.g. grep -rn "CGAL::" ${REF}). Be exact and minimal. Return the structured audit.`,
    { schema: AUDIT_SCHEMA, phase: 'Audit', label: 'native-audit' }
  )

  phase('Critique')
  const allCpp = inv.chunks.flatMap((c) => c.files.map((f) => f.cpp))
  const critic = await agent(
`You are the COMPLETENESS CRITIC for this parity MAP. Be adversarial — find what was missed.

- Full C++ file list expected (${allCpp.length}): ${JSON.stringify(allCpp)}
- Files actually classified (${records.length}): ${JSON.stringify(records.map((r) => r.cpp))}
- Rust orphans reported: ${JSON.stringify(inv.rustOrphans)}
- Native libs in records: ${JSON.stringify(nativeLibSet)}
- Native libs covered by audit: ${JSON.stringify((audit && audit.libraries || []).map((l) => l.name))}

Verify via Bash if needed (e.g. re-find C++ files under ${REF}). Report: unmappedCpp (in the expected list but not classified), unexplainedRustOrphans, auditGaps (libs in records but absent from audit), inconsistencies (e.g. status=done but structuralFidelity=diverges, or rust=null but status!=missing), an overallCompleteness 0..1 (priority-weighted estimate of how done the existing Rust port is), and a one-paragraph verdict on the biggest risks to achieving EXACT gcode parity.`,
    { schema: CRITIC_SCHEMA, phase: 'Critique', label: 'completeness-critic' }
  )

  // Sort records: missing/stub/partial first, by priority then effort, for the roadmap.
  const statusRank = { missing: 0, stub: 1, partial: 2, done: 3 }
  const prioRank = { critical: 0, high: 1, medium: 2, low: 3 }
  const sorted = records.slice().sort((a, b) =>
    (statusRank[a.status] - statusRank[b.status]) ||
    (prioRank[a.priority] - prioRank[b.priority]) ||
    (b.cppLoc - a.cppLoc))

  return {
    phase: 'map',
    totals: inv.totals,
    counts: { status: statusCounts, structuralFidelity: fidelityCounts, priority: priorityCounts, filesUsingNative: nativeFiles.length },
    records: sorted,
    rustOrphans: inv.rustOrphans,
    nativeAudit: audit,
    critic,
  }
}

// ============================================================================
// PHASE: PORT  (bounded batch; sequential edits in main tree so they accumulate)
// ============================================================================
async function runPort() {
  const items = (args && args.items) ? args.items : []
  if (!items.length) throw new Error('PORT needs args.items = [repo-root-relative .cpp paths]. Run MAP first and pick from PARITY.json.')
  phase('Port')
  const out = []
  for (let i = 0; i < items.length; i++) {
    const cpp = items[i]
    const ported = await agent(
`Faithfully port ONE C++ file to Rust in crate ${CRATE}. Goal: a 1:1 translation that mirrors the C++ structure (same functions, same order, same control flow, same names in snake_case) — like Bun's Zig->Rust rewrite. EXACT behavioral parity; do not "improve" logic.

C++ source: ${cpp}
Target Rust file: derive by the project's PascalCase->snake_case+subdir convention; create it under ${SRC} if missing, else extend it. Match the module wiring in ${SRC}/lib.rs.
Read neighboring already-ported files to match idioms/types (Point, Polygon, ExPolygon, coord_t, etc.). Preserve comments. Port every public symbol. Use existing crate types; do not introduce native/system linkage (must stay wasm-safe).
Return what changed + any symbols you could not yet port (with reason).`,
      { schema: PORT_SCHEMA, phase: 'Port', label: `port:${cpp.split('/').pop()}` }
    )
    if (!ported) { out.push({ cpp, skipped: true }); continue }
    const review = await agent(
`Adversarially review this port for FAITHFULNESS to the C++ original. Compare line-by-line / block-by-block.
C++: ${cpp}
Rust: ${ported.rust}
Open BOTH. For every C++ function, confirm the Rust mirrors its logic, branch order, rounding, integer/float types, and edge cases (off-by-one, <=vs<, clipper fill rules, coordinate scaling). Flag ANY divergence that could change gcode output. Default to faithful=false if uncertain. fidelityScore 0..1.`,
      { schema: REVIEW_SCHEMA, phase: 'Review', label: `review:${cpp.split('/').pop()}` }
    )
    const build = await agent(
`Verify the crate still compiles after porting ${cpp}. IMPORTANT: builds MUST run inside devbox — run exactly: devbox run cargo build -p slicer 2>&1 | tail -80 (from repo root). Do NOT run bare cargo. Report compiles + any error strings. Do not fix unrelated code; only report.`,
      { schema: BUILD_SCHEMA, phase: 'Build', label: `build:${cpp.split('/').pop()}` }
    )
    out.push({ cpp, rust: ported.rust, ported, review, build })
    log(`${i + 1}/${items.length} ${cpp}: faithful=${review && review.faithful} compiles=${build && build.compiles}`)
  }
  return { phase: 'port', results: out }
}

// ============================================================================
// PHASE: VERIFY (gcode parity)
// ============================================================================
async function runVerify() {
  phase('Verify')
  const r = await agent(
`Run the integration/parity tests for exact gcode parity between the Rust slicer and the C++ BambuStudio reference. Builds/tests MUST run inside devbox.
Steps (from repo root):
- devbox run native:build (build the reference C++ binary) if needed.
- Run: SLICER_CLI_TEST_MODE=devbox bun test tests  (capture summary).
- Also run the crate's own integration tests: devbox run cargo test -p slicer 2>&1 | tail -60.
- Where a generated gcode differs from reference, use the 'validate' subcommand / diff to characterize the first divergence (layer, line, command) and trace it to the responsible module.
Return a concise report of pass/fail counts and the top divergences with the module each points to (so they become PORT work items).`,
    { schema: { type: 'object', additionalProperties: false, required: ['summary', 'passed', 'failed', 'divergences'], properties: { summary: { type: 'string' }, passed: { type: 'integer' }, failed: { type: 'integer' }, divergences: { type: 'array', items: { type: 'object', additionalProperties: false, required: ['test', 'firstDiff', 'suspectModule'], properties: { test: { type: 'string' }, firstDiff: { type: 'string' }, suspectModule: { type: 'string' } } } } } }, phase: 'Verify', label: 'parity-verify' }
  )
  return { phase: 'verify', report: r }
}

// ----------------------------------------------------------------------------
if (phaseArg === 'map') return await runMap()
if (phaseArg === 'port') return await runPort()
if (phaseArg === 'verify') return await runVerify()
throw new Error(`Unknown phase: ${phaseArg} (expected map|port|verify)`)