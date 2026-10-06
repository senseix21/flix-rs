FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends git build-essential cmake pkg-config libgazebo-dev gazebo python3 curl ca-certificates && rm -rf /var/lib/apt/lists/*
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup.sh && sh /tmp/rustup.sh -y --default-toolchain 1.92.0 --profile minimal
ENV PATH=/root/.cargo/bin:$PATH
WORKDIR /work
