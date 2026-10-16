#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
python3 scripts/generate.py --check
cargo fmt --all --check
cargo fmt --manifest-path apps/esp32/firmware/Cargo.toml -- --check
cargo fmt --manifest-path apps/proxy/firmware/Cargo.toml -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo clippy -p flight-proxy --all-targets --features host --locked -- -D warnings
# Firmware bridges hold the audited unsafe FFI; lint them on a portable no_std target.
cargo clippy --manifest-path apps/esp32/firmware/Cargo.toml --target thumbv7em-none-eabihf --locked --release -- -D warnings
cargo clippy --manifest-path apps/proxy/firmware/Cargo.toml --target thumbv7em-none-eabihf --locked --release -- -D warnings
cargo test --workspace --locked
cargo build --workspace --locked --release
cargo build -p flight-proxy --features host --locked --release
cargo check -p flight-core -p flight-protocol -p flight-config -p flight-esp32 --locked --release
PYTHONPATH=tools python3 -m unittest discover -s tools/tests -v
python3 scripts/legacy-parity.py
python3 scripts/abi-smoke.py
# Firmware UDP transport on the host: endpoint learning follows owner admission (R2-05).
sh scripts/radio-endpoint-test.sh
# Swarm S1 harness: real owners through the cdylib, no ROS (swarm plan S1-09, S1-10).
PYTHONPATH=adapters/crazyswarm2 python3 -m unittest discover -s adapters/crazyswarm2/tests -v
PYTHONPATH=adapters/crazyswarm2 python3 -m flix_swarm.scenarios --output target/swarm-s1
PYTHONPATH=adapters/crazyswarm2 python3 -m flix_swarm.scenarios --onboard --output target/swarm-s2
python3 scripts/simulator-contracts.py
cargo run --locked --release -p flight-replay --bin flight-replay -- --benchmark
git diff --check
