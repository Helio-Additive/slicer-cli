# Source code, licences and acknowledgements

Helio's slicing tools use **slicer-cli**, a command-line program built from
Bambu Studio, OrcaSlicer and Snapmaker Orca. Its public source is available
without a Helio account. This includes the slicer functionality accessed
through Helio plugins, whether slicing a model or processing existing G-code.

- [slicer-cli source](https://github.com/Helio-Additive/slicer-cli)
- [AGPL licence text](../LICENSE)
- [Acknowledgements and dependency notices](../NOTICE)
- [Releases and downloads](https://github.com/Helio-Additive/slicer-cli/releases)
- [Build instructions](building.md)
- [Helio Additive website](https://www.helioadditive.com)

These are the notices for the slicer software. They do not designate the
licence of a plugin package or of Dragon's thermal simulation service.

## Get the source for the version you used

Use the slicer-cli release identified by your service or downloaded package.
The latest release or the current `main` branch may differ from the version
that processed a particular job. If the service has not supplied its release
identifier, request it from the service operator rather than guessing.

Each release uses a tag of the form `slicer-cli/v<version>`. Its
`engine-manifest.json` asset records the release `version` and CLI `gitSha`.
The gitlinks in that commit record the exact engine revisions; [`.gitmodules`](../.gitmodules)
records their public repositories. The per-platform manifest and packaged
notices describe the binaries and libraries included in that platform's archive.

For example, to retrieve **v0.1.0-dev-22** (replace the tag with the release
you actually used):

```sh
git clone --no-checkout https://github.com/Helio-Additive/slicer-cli.git
cd slicer-cli
git checkout --detach slicer-cli/v0.1.0-dev-22
git submodule update --init --recursive
git rev-parse HEAD
git submodule status --recursive
```

Compare `git rev-parse HEAD` with the release manifest's `gitSha`. Read
`docs/building.md` and `.github/workflows/slicer-cli-ci.yml` **in that checkout**
for its build instructions. The checkout contains:

- the CLI front end and build configuration;
- the pinned engines under `references/`, including their own notices;
- Helio's engine overrides under `libslic3r/`;
- dependency patches and overlay ports under `ci/`;
- `install_deps.sh` and the release workflow, which specify the external
  dependency sources and build steps for each platform.

GitHub's automatic **Source code (zip/tar.gz)** downloads do not populate Git
submodules. Use the recursive checkout above to obtain the pinned engines as
well. The override files and patches are part of slicer-cli; an upstream
engine checkout on its own does not contain Helio's changes.

## Acknowledgements

| Component | Project source | Relationship |
| --- | --- | --- |
| Helio slicer-cli | [Helio-Additive/slicer-cli](https://github.com/Helio-Additive/slicer-cli) | CLI front end, engine overrides and packaging |
| Bambu Studio | [bambulab/BambuStudio](https://github.com/bambulab/BambuStudio) | Bambu engine; based on PrusaSlicer and Slic3r |
| OrcaSlicer | [OrcaSlicer/OrcaSlicer](https://github.com/OrcaSlicer/OrcaSlicer) | Orca engine; based on Bambu Studio and its predecessors |
| Snapmaker Orca | [Snapmaker/OrcaSlicer](https://github.com/Snapmaker/OrcaSlicer) | Snapmaker engine; based on OrcaSlicer |
| PrusaSlicer | [prusa3d/PrusaSlicer](https://github.com/prusa3d/PrusaSlicer) | Upstream slicer and geometry-processing contributions |
| Slic3r | [slic3r/Slic3r](https://github.com/slic3r/Slic3r) | Original slicer project |

The linked projects retain their copyright and licence notices. For the code
in a particular build, consult the files at its pinned revisions, rather than
the current upstream default branches.

## Third-party libraries

The engines also use third-party libraries. [NOTICE](../NOTICE) lists selected
components and their licences. Release archives contain `LICENSE`, `NOTICE`,
the three engine licence texts, and `THIRD_PARTY_LICENSES/` with the available
platform dependency records. Those records and the dependency build recipes
in the release checkout should be read together; the libraries differ across
platforms and releases. This page is not a claim that a complete dependency
licence audit has been performed.

## Source access from hosted services

Services using this software should make this source link visible to users of
the slicer functionality and identify their deployed release. AGPL section 13
is in [LICENSE](../LICENSE); the GNU project's explanation of indirect network
interaction is in its [AGPL proxy FAQ](https://www.gnu.org/licenses/gpl-faq.html#AGPLProxy).
The company website and a plugin's own licence metadata are separate from
this offer of slicer source.
