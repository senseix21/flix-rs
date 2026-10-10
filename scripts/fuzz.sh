#!/bin/sh
# Run every fuzz target for SECONDS (default 60). Requires a nightly toolchain and
# cargo-fuzz. Crashes are written to fuzz/artifacts/<target>/ and fail the run.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
seconds=${1:-60}
cd "$root/fuzz"
for target in $(cargo +nightly fuzz list); do
    mkdir -p "corpus/$target" "seeds/$target"
    echo "== $target (${seconds}s)"
    cargo +nightly fuzz run "$target" "corpus/$target" "seeds/$target" -- \
        -max_total_time="$seconds" -print_final_stats=1
done
