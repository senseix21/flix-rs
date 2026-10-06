#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
: "${IDF_PATH:?Set IDF_PATH to the ESP-IDF v5.4.2 checkout}"
test "$(git -C "$IDF_PATH" describe --tags --exact-match)" = "v5.4.2"
cargo +esp --version
rustc +esp --version | rg '^rustc 1\.88\.0'
cargo +esp rustc --manifest-path "$root/apps/esp32/firmware/Cargo.toml" --locked --release --target xtensa-esp32s3-espidf -Z build-std=core -- -C panic=abort
cd "$root/apps/esp32/idf"
# Preserve the operator's reviewed sdkconfig on repeat builds.
if test -f sdkconfig; then
    rg -q '^CONFIG_IDF_TARGET="esp32s3"$' sdkconfig || { echo "sdkconfig must target esp32s3" >&2; exit 1; }
else
    idf.py set-target esp32s3
fi
idf.py build
