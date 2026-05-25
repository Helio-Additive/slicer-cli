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

# Run the bundled Benchy example.
example:
    devbox run example

# Build the Docker image with the Rust CLI and native slicer binary.
image:
    docker buildx bake slicer-cli
