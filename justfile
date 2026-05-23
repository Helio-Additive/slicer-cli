configure:
    cmake -S . -B build -DCMAKE_PREFIX_PATH="${LIBNOISE_PREFIX:-}"

build: configure
    cmake --build build --parallel
    cargo build

# Run all tests.
test:
    cargo test

image:
    docker buildx bake

example:
    bash {{ justfile_directory() }}/example/run.sh
