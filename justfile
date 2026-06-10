set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default: build

setup:
    devbox install

# Build the Rust CLI and native BambuStudio slicer binary.
build: setup
    devbox run native:build
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

# Build the Docker image with the Rust CLI and native slicer binary.
image:
    docker buildx bake slicer-cli

tests: setup
    devbox run native:build
    SLICER_CLI_TEST_MODE=devbox bun test tests

tests-docker: setup
    docker buildx bake slicer-cli
    SLICER_CLI_TEST_MODE=docker SLICER_CLI_TEST_IMAGE=slicer-cli:local bun test tests
