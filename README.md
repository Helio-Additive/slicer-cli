# slicer-cli

Standalone dual-engine command-line slicer derived from BambuStudio and
OrcaSlicer's `libslic3r` forks. AGPLv3.

`slicer-cli` is the AGPL boundary that closed apps subprocess. Run it directly,
embed it in CI, or wrap it from any tool that wants reproducible BambuStudio-
compatible STL → gcode slicing without dragging in a UI.

## License

AGPL-3.0-or-later. See `LICENSE`.

This project includes separate BambuStudio and OrcaSlicer engine binaries. Both
are derived from PrusaSlicer and Slic3r under AGPLv3. The full attribution chain
and engine-specific dependency provenance are in `NOTICE`.

If you run `slicer-cli` (or a modified version of it) on a server and let
network users interact with it, AGPL § 13 requires you to offer them the
source. The recommended pattern is a "Source" link in the UI of whatever
service wraps the binary. The Helio closed apps wrap `slicer-cli` via
subprocess invocation only — no AGPL linking — so their UIs are not bound by
§ 13; this README is for direct downstream consumers who are.

## What it does

- Reads STL or 3MF input.
- Resolves a printer / filament / process profile triple (BBL profile schema —
  same as BambuStudio).
- Generates G-code byte-identical to what BambuStudio Desktop would produce
  for the same inputs (within the supported-feature matrix below).

## What it does NOT do

This is `libslic3r`-headless. The following upstream features are intentionally
excluded; full list with rationale and user-visible behaviour in
`docs/slicer-cli-supported-features.md`:

- SLA hollowing (`OpenVDBUtils.cpp`, `SLA/Hollowing.cpp`)
- Mesh cutting (`CutSurface.cpp` — incompatible with system CGAL 6.x)
- User post-processor scripts (`PostProcessor.cpp` — security-sensitive,
  out of v1 scope)
- Network printer push (`GCodeSender.cpp`)
- Pressure equalizer (`PressureEqualizer.cpp`)

Any of these surfacing as a CLI flag returns a clear capability error rather
than silently mis-processing. A regression test per excluded feature pins the
behaviour.

## Build

Dependencies are system packages — see `install_deps.sh`.

```sh
git clone --recurse-submodules https://github.com/<org>/slicer-cli.git
cd slicer-cli
./install_deps.sh        # installs Homebrew / apt packages
mkdir -p cli/build && cd cli/build
cmake ..
cmake --build . -j
./slicer_cli --help
```

Submodule pin: `references/BambuStudio` is pinned at a known-good commit
(see `.gitmodules`). Bumping the pin is a deliberate maintenance action —
it can ripple through the override layer and the patched libigl tree. Test on
a feature branch first.

## Releases

Release builds are produced by GitHub Actions (`.github/workflows/`) for:

- Linux x86_64
- macOS arm64
- Windows x86_64

Linux packages target Ubuntu 22.04/glibc 2.35 and bundle non-glibc runtime
libraries. macOS packages bundle non-system dylibs and reject Homebrew or build
paths. CI artifacts are not currently notarized or Authenticode-signed.

The engine dependency contract, upstream-aligned pins, and documented platform
exceptions are recorded in `docs/engine-dependency-contract.md`.

Package metadata identifies the artefact as `slicer_cli` (not `BambuStudio`).
A CI assertion fails the build if the metadata regresses.

## AWS Lambda worker

`lambda-worker/` contains a Rust Lambda custom runtime that downloads a queued
STL/3MF from S3, executes the packaged slicer as a subprocess, uploads G-code
and a result checkpoint to S3, then calls the regular API. The worker consumes
SQS with batch size one; the supplied SAM template uses a FIFO queue so a
duplicate job cannot slice concurrently and a callback retry reuses the stored
result instead of slicing again.

The API must send each message with `MessageGroupId` set to `jobId` and
`MessageDeduplicationId` set to a stable hash of the complete request body.
This serializes retries of one job without serializing unrelated jobs.

The versioned, code-generation-ready contract is exported as
[`schemas/json/exported/slice-request.schema.json`](schemas/json/exported/slice-request.schema.json).
The event must validate against that schema before the API enqueues it.

The API places this internal message on the queue after the input upload has
completed:

```json
{
  "schemaVersion": 1,
  "jobId": "01J9EXAMPLE",
  "input": {
    "bucket": "helio-slicer-input-production",
    "key": "jobs/01J9EXAMPLE/model.3mf",
    "versionId": "optional-s3-version",
    "sha256": "64-lowercase-or-uppercase-hex-characters"
  },
  "output": {
    "bucket": "helio-slicer-output-production",
    "key": "jobs/01J9EXAMPLE/output.gcode"
  },
  "engine": "bambu",
  "profiles": {
    "mode": "embedded"
  },
  "options": {
    "plate": 1,
    "overrides": {
      "layerHeightMm": 0.2,
      "infillPercent": 20,
      "perimeters": 3
    }
  }
}
```

STL jobs use `"mode": "explicit"` and supply `machine`, `filament`, and
`process` paths relative to the packaged `slicer-cli` directory. The regular
API must map its public profile IDs to these paths; public callers must never
submit package paths directly. An optional `config` object uses the same
`bucket`/`key`/`versionId`/`sha256` shape as `input`, for example an inherited
Orca configuration resolved by the API. Request bucket names are required and
must exactly match the worker's configured input and output buckets.

The callback sends `POST $CALLBACK_URL` with the exact stored result JSON and:

- `X-Helio-Event: slice.finished`
- `Idempotency-Key: <jobId>`
- `X-Helio-Timestamp: <Unix seconds>`
- `X-Helio-Signature: v1=<hex HMAC-SHA256>`

The signed bytes are:

```text
<X-Helio-Timestamp>.<raw request body>
```

The API must verify the signature in constant time, reject stale timestamps,
and process `jobId` idempotently. A non-2xx response is retried three times;
continued callback failure returns the SQS message for retry. Successful and
permanent-failure results are checkpointed beside the output as
`<output-key>.result.json`, so callback retries do not rerun the slicer.

To build the image, extract the verified Linux release archive so that
`lambda-worker/.package/slicer-cli/bin/slicer_cli` exists, then run:

```sh
docker build --platform linux/amd64 --target production -t slicer-lambda lambda-worker
```

`lambda-worker/template.yaml` creates private versioned input/output buckets,
an encrypted FIFO queue and DLQ, the image-based function, least-privilege S3
access, and baseline alarms. CI smoke-tests the image. Pushes to `main` deploy
staging and release tags deploy production when their GitHub environments
provide:

- variables `AWS_DEPLOY_ROLE_ARN`, `AWS_REGION`,
  `SLICER_LAMBDA_ECR_REPOSITORY`, `SLICER_LAMBDA_STACK_NAME`, and
  `SLICER_CALLBACK_URL`
- secret `SLICER_CALLBACK_HMAC_SECRET` (at least 32 bytes)

The ECR repository and OIDC deployment role are bootstrap resources and must
exist before the first deployment.

## Using a release package

Choose the engine by choosing its binary: `slicer_cli` uses BambuStudio;
`slicer_cli-orcaslicer` uses OrcaSlicer. Run the examples from the extracted
`slicer-cli` directory, using profiles from the matching engine's tree.
On Linux the binaries live under `bin/` inside the extracted `slicer-cli`
directory (`./bin/slicer_cli`, `./bin/slicer_cli-orcaslicer`) and nothing
launches from that directory's top level; the macOS and Windows archives keep
their flat layout with the binaries directly inside `slicer-cli`. On Windows,
use the corresponding `.exe` filename.

On every platform, the BambuStudio package reconstructs named presets from
the bundled profiles when reading a Bambu 3MF (the archive ships the `BBL`
profile tree and its `BBL.json` vendor index, and each package job slices a
Bambu 3MF from the extracted archive and requires the three presets to
resolve):

```sh
./bin/slicer_cli model.3mf -o bambu.gcode          # Linux
./slicer_cli model.3mf -o bambu.gcode              # macOS / Windows (.exe)
```

Both engines also read their slice-time resource folders from the package:
the Bambu engine `resources/info`, `resources/flush` and
`resources/filament_mixing`; the Orca engine its own `resources/orca/info`
and `resources/orca/flush`. Each engine prints the root it configured on
stderr at startup (`Engine resources: …`), except under `--layout-plan`,
whose stdout and stderr are JSON documents.

For OrcaSlicer, this example selects the packaged Snapmaker U1 profiles
(shown with the Linux `bin/` path; on macOS and Windows run
`./slicer_cli-orcaslicer` from the extracted `slicer-cli` directory).
First supply `resolved-orca-config.json` containing their complete inherited
settings. The caller must resolve the profiles' `inherits` chains: the CLI
loads JSON overrides directly and does not resolve those chains itself.
Passing only the leaf files below would leave parent settings at defaults.

```sh
./bin/slicer_cli-orcaslicer model.stl \
  --config resolved-orca-config.json \
  --machine 'resources/profiles-orca/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json' \
  --filament 'resources/profiles-orca/Snapmaker/filament/Snapmaker PLA @U1.json' \
  --process 'resources/profiles-orca/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json' \
  -o orca.gcode
```

Use profiles matching your actual printer, nozzle, and material before printing.

## Versioning

`slicer-cli` uses semver. The major version may bump when the supported-
feature matrix narrows. The version reported in gcode headers is the pinned
BambuStudio submodule version + a `slicer-cli/<version>` suffix so you can
tell which lineage produced the output.

## Contributing

See `CONTRIBUTING.md`. External PRs require signing the CLA (preserves
dual-licence optionality) and a green CI build on all three platforms.

## Security

See `SECURITY.md` for the disclosure policy.

## Related projects

`slicer-cli` is one of two repos in the Helio source release:

- **`slicer-cli` (this repo, public AGPLv3)** — the engine.
- **`helio-platform` (private, commercial)** — the Tauri client, the Rust
  MCP server, the React UI, the rust-server cloud deploy, the Helio agent
  intelligence. Subprocess-invokes `slicer-cli`. No linking, no AGPL
  propagation.

This separation is intentional: AGPLv3 stays pinned to the engine where it
actually matters; closed-product code stays closed without cross-contaminating
licences.
