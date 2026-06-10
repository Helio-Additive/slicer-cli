# syntax=docker/dockerfile:1.7

# Build with the same pinned toolchain used for local development.
# The Docker build context must include initialized submodules:
#   git submodule update --init --recursive
FROM jetpackio/devbox:0.17.2 AS devbox-base

SHELL ["/bin/bash", "-euo", "pipefail", "-c"]

WORKDIR /workspace

COPY devbox.json devbox.lock ./
RUN devbox install

FROM devbox-base AS cargo-deps

COPY Cargo.toml Cargo.lock ./
RUN --mount=type=cache,target=/home/devbox/.cargo/registry,uid=1000,gid=1000 \
    --mount=type=cache,target=/home/devbox/.cargo/git,uid=1000,gid=1000 \
    devbox run -- cargo fetch --locked

FROM cargo-deps AS builder

COPY . .
RUN --mount=type=cache,target=/home/devbox/.cargo/registry,uid=1000,gid=1000 \
    --mount=type=cache,target=/home/devbox/.cargo/git,uid=1000,gid=1000 \
    --mount=type=cache,target=/workspace/target,uid=1000,gid=1000 \
    --mount=type=cache,target=/workspace/libslic3r/bambustudio/build,uid=1000,gid=1000 \
    devbox run native:build \
    && devbox run cargo build --release \
    && mkdir -p /workspace/artifacts/bin /workspace/artifacts/rootfs \
    && cp target/release/slicer-cli /workspace/artifacts/bin/slicer-cli \
    && cp libslic3r/bambustudio/build/slicer_cli /workspace/artifacts/bin/slicer_cli \
    && for binary in /workspace/artifacts/bin/slicer-cli /workspace/artifacts/bin/slicer_cli; do \
      ldd "$binary"; \
    done \
    | awk '$2 == "=>" && $3 ~ /^\// { print $3 } $1 ~ /^\// { print $1 }' \
    | sort -u \
    | while read -r lib; do \
      mkdir -p "/workspace/artifacts/rootfs$(dirname "$lib")"; \
      cp -L "$lib" "/workspace/artifacts/rootfs$lib"; \
    done

FROM debian:stable-slim AS cert-builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

FROM gcr.io/distroless/static-debian12:nonroot AS runtime

COPY --from=cert-builder /etc/ssl/certs/ca-certificates.crt /etc/ssl/certs/ca-certificates.crt
COPY --from=builder /workspace/artifacts/rootfs/ /
COPY --from=builder /workspace/artifacts/bin/slicer-cli /bin/slicer-cli
COPY --from=builder /workspace/artifacts/bin/slicer_cli /bin/slicer_cli
COPY --from=builder /workspace/libslic3r/bambustudio/references/BambuStudio/resources/profiles/ /profiles/

ENV BAMBUSTUDIO_SLICER=/bin/slicer_cli
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt

WORKDIR /workspace/data

ENTRYPOINT ["/bin/slicer-cli"]
CMD ["--help"]
