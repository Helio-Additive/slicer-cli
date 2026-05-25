# syntax=docker/dockerfile:1.7

# Build with the same pinned toolchain used for local development.
# The Docker build context must include initialized submodules:
#   git submodule update --init --recursive
FROM jetpackio/devbox:0.17.1 AS builder

SHELL ["/bin/bash", "-euo", "pipefail", "-c"]

WORKDIR /workspace

COPY devbox.json devbox.lock ./
RUN devbox install

COPY Cargo.toml Cargo.lock ./
RUN --mount=type=cache,target=/root/.cargo/registry \
    --mount=type=cache,target=/root/.cargo/git \
    devbox run -- cargo fetch --locked

COPY . .

ENV RUSTFLAGS="-C target-feature=+crt-static"

RUN --mount=type=cache,target=/root/.cargo/registry \
    --mount=type=cache,target=/root/.cargo/git \
    --mount=type=cache,target=/workspace/target \
    --mount=type=cache,target=/workspace/libslic3r/bambustudio/build \
    devbox run -- just build \
    && mkdir -p /artifacts/bin \
    && cp target/release/slicer-cli /artifacts/bin/slicer-cli \
    && cp libslic3r/bambustudio/build/slicer_cli /artifacts/bin/slicer_cli \
    && for binary in /artifacts/bin/slicer-cli /artifacts/bin/slicer_cli; do \
      ldd_output="$(ldd "$binary" 2>&1 || true)"; \
      if echo "$ldd_output" | grep -q "=>"; then \
        echo "$binary is dynamically linked, but the runtime image is distroless/static"; \
        echo "$ldd_output"; \
        exit 1; \
      fi; \
    done

FROM debian:stable-slim AS cert-builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && update-ca-certificates \
    && rm -rf /var/lib/apt/lists/*

FROM gcr.io/distroless/static-debian12:nonroot AS runtime

COPY --from=cert-builder /etc/ssl/certs/ca-certificates.crt /etc/ssl/certs/ca-certificates.crt
COPY --from=builder /artifacts/bin/slicer-cli /bin/slicer-cli
COPY --from=builder /artifacts/bin/slicer_cli /bin/slicer_cli
COPY --from=builder /workspace/libslic3r/bambustudio/references/BambuStudio/resources/profiles/ /profiles/

ENV BAMBUSTUDIO_SLICER=/bin/slicer_cli
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt

WORKDIR /workspace/data

ENTRYPOINT ["/bin/slicer-cli"]
CMD ["--help"]
