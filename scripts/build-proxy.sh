#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
: "${IDF_PATH:?Set IDF_PATH to ESP-IDF v5.4.2}"
test "$(git -C "$IDF_PATH" describe --tags --exact-match)" = v5.4.2
rustc +esp --version | rg '^rustc 1\.88\.0'
cargo +esp rustc --manifest-path "$root/apps/proxy/firmware/Cargo.toml" --locked --release --target xtensa-esp32s3-espidf -Z build-std=core -- -C panic=abort
cd "$root/apps/proxy/idf"
if test -f sdkconfig; then
    rg -q '^CONFIG_IDF_TARGET="esp32s3"$' sdkconfig
else
    idf.py set-target esp32s3
fi
idf.py build
