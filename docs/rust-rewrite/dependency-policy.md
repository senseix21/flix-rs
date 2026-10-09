# Dependency and build policy

The portable graph pins `libm=0.2.16` and `sha2=0.10.9`; both manifests commit a Cargo.lock. `sha2` has no default features. No generated full MAVLink library, async runtime, heap queue or hardware feature enters the flight core. Host crates use the standard library; SDK/host startup allocations are outside the control tick allocation contract.

The ESP32 boundary pins Rust Xtensa v1.88.0.0, ESP-IDF v5.4.2 and its submodule revisions. The local setup used espup v0.16.0. SDK Python tools follow Espressif's v5.4 constraints; save the installed tool/dependency manifest with an artifact. The Gazebo verification image uses Ubuntu 22.04, Gazebo 11 and Rust 1.92.0. Record the resolved image digest and package versions as evidence; the Dockerfile alone does not freeze every apt package.

Before a distributable release, review the lockfile's advisory database, dependency licenses and legacy provenance. No advisory scan or licensing approval follows from a compiler pass. No remote publication is part of this development task.

`proptest=1.11.0` (no default features, `std` only) is a host dev-dependency of `flight-core` for the model-based safety properties. It is absent from the normal dependency graph and never linked into firmware; `cargo tree -p flight-core -e normal` still lists only `libm`.
