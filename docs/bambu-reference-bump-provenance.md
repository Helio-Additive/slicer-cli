# BambuStudio reference bump provenance

## Source pair

- Previous locked BambuStudio reference: `b506005bc4ee62124e24bf00e0f58656db3646a6`.
- Bump candidate: `ba049f6a2e08c3b6033660bb84da80c08722974b`.
- The candidate includes the post-`2f014ce1` consolidated filament-grouping
  implementation being evaluated by the Wave 5 acceptance suite.

## Build-dependency delta

The reference source now requires Assimp through `libslic3r` (`find_package(assimp
REQUIRED)` and `assimp::assimp`). The CLI owns that dependency rather than relying
on BambuStudio's private dependency build.

| Platform | Provisioning change |
| --- | --- |
| macOS | Homebrew `assimp` in `install_deps.sh`; Homebrew cache/repair marker advanced to v4. |
| Debian/Ubuntu | `libassimp-dev` in `install_deps.sh` and the Linux CI install step. |
| Windows | `assimp:x64-windows` in the pinned-vcpkg CI install step; vcpkg cache advanced to v2. |

`libnoise` is **not** part of this bump's dependency delta. It was already built
from Bambu's fork by `install_deps.sh` and all three CI platforms before this
reference change.

The bump candidate also incorporates the stale-Homebrew-cache repair from
`b82aeee`: skip the installed-dependents walk and remove formula records whose
Cellar keg is missing before the declared dependency set is installed. This is
CI cache hardening, not an engine build dependency.

## Local validation scope

This provenance record tracks a local, unmerged source-bump investigation only.
It does not identify a released engine, alter a runtime lock, or authorize a
remote push. Release metadata and platform binary hashes will be recorded only
after the CLI bump is reviewed, merged by Priyesh, built by macOS and Windows CI,
and published under the normal release process.
