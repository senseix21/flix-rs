#!/bin/sh
# Supply-chain gate: advisories, licences, bans and sources for every locked
# workspace. Needs cargo-deny and network access for the RustSec database.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
for manifest in Cargo.toml apps/esp32/firmware/Cargo.toml apps/proxy/firmware/Cargo.toml fuzz/Cargo.toml; do
    echo "== $manifest"
    cargo deny --manifest-path "$manifest" --config deny.toml --locked check
done
