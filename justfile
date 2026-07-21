set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default: build

setup:
    devbox install

# Build the Rust CLI and bambu (C++ BambuStudio) slicer binary.
build: setup
    devbox run bambu:build
    devbox run cargo build --release

package: setup
    devbox run package

format:
    devbox run cargo fmt
    devbox run bash -eu -o pipefail -c 'git ls-files "*.c" "*.cc" "*.cpp" "*.cxx" "*.h" "*.hh" "*.hpp" "*.hxx" | grep -Ev "^(libslic3r/bambustudio/references/|libslic3r/bambustudio/build/)" | xargs -r clang-format -i'
    devbox run bash -eu -o pipefail -c 'cargo clippy --all-targets -- -D warnings'

format-check:
    devbox run cargo fmt -- --check
    devbox run bash -eu -o pipefail -c 'git ls-files "*.c" "*.cc" "*.cpp" "*.cxx" "*.h" "*.hh" "*.hpp" "*.hxx" | grep -Ev "^(libslic3r/bambustudio/references/|libslic3r/bambustudio/build/)" | xargs -r clang-format --dry-run --Werror'
    devbox run bash -eu -o pipefail -c 'cargo clippy --all-targets -- -D warnings'

# Run the bundled Benchy example with either the local devbox build or Docker image.
example target="devbox":
    bun install
    mkdir -p _downloads
    if [ ! -f _downloads/3DBenchy.stl ]; then curl -L --fail --show-error --output _downloads/3DBenchy.stl https://helioadditive-public.s3.ap-east-1.amazonaws.com/3DBenchy.stl; fi

    @case "{{ target }}" in \
        devbox) devbox run example ;; \
        docker) docker run --rm --volume "$PWD:$PWD" --workdir "$PWD" slicer-cli:local slice --config examples/config.jsonnet ;; \
        *) echo "unknown example target: {{ target }} (expected devbox or docker)" >&2; exit 2 ;; \
    esac

# Build the Docker image with the Rust CLI and bambu slicer binary.
image:
    docker buildx bake slicer-cli

tests: setup
    devbox run bambu:build
    SLICER_CLI_TEST_MODE=devbox bun test tests

tests-docker: setup
    docker buildx bake slicer-cli
    SLICER_CLI_TEST_MODE=docker SLICER_CLI_TEST_IMAGE=slicer-cli:local bun test tests

# Slice every tests/configs job config through BOTH engines (bambu = C++ libslic3r,
# rust = in-process Rust) into a timestamped tests/.tmp/<datetime>/ tree, one
# subdir per config with both G-codes side by side. Pure artifacts, no asserts.
# Restrict to specific configs: `just slice-configs stl-file-config nu3mf`.
slice-configs *configs: setup
    devbox run bambu:build
    devbox run cargo build
    mkdir -p _downloads
    if [ ! -f _downloads/3DBenchy.stl ]; then curl -L --fail --show-error --output _downloads/3DBenchy.stl https://helioadditive-public.s3.ap-east-1.amazonaws.com/3DBenchy.stl; fi
    devbox run bun run scripts/slice-configs.ts {{ configs }}
