FROM rust:1.95-bookworm AS worker-builder
RUN apt-get update \
    && apt-get install -y --no-install-recommends clang cmake musl-tools \
    && rm -rf /var/lib/apt/lists/* \
    && rustup target add x86_64-unknown-linux-musl
WORKDIR /build
COPY Cargo.toml Cargo.lock ./
COPY src/ src/
RUN cargo build --locked --release --target x86_64-unknown-linux-musl \
    && cp target/x86_64-unknown-linux-musl/release/slicer-lambda /bootstrap

FROM ubuntu:22.04 AS runtime
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=worker-builder /bootstrap /bootstrap
COPY .package/slicer-cli/ /opt/slicer-cli/
RUN chmod 0555 /bootstrap \
        /opt/slicer-cli/bin/slicer_cli \
        /opt/slicer-cli/bin/slicer_cli-orcaslicer \
    && test -f /opt/slicer-cli/bin/manifest.json \
    && test -d /opt/slicer-cli/resources/profiles/BBL
ENV HOME=/tmp \
    TMPDIR=/tmp \
    SLICER_ROOT=/opt/slicer-cli \
    RUST_LOG=info
ENTRYPOINT ["/bootstrap"]

FROM runtime AS production

FROM public.ecr.aws/lambda/provided:al2023 AS lambda-runtime-tools

FROM runtime AS local
COPY --from=lambda-runtime-tools /usr/local/bin/aws-lambda-rie /usr/local/bin/aws-lambda-rie
COPY local-entrypoint.sh /local-entrypoint.sh
RUN chmod 0555 /usr/local/bin/aws-lambda-rie /local-entrypoint.sh
ENTRYPOINT ["/local-entrypoint.sh"]
