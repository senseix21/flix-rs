#!/bin/sh
# R2-05: build the firmware's UDP radio transport on the host and check endpoint learning.
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
out="$root/target/radio-endpoint-test"
mkdir -p "$out"
${CC:-cc} -std=c11 -D_DEFAULT_SOURCE -D_DARWIN_C_SOURCE -Wall -Wextra -Werror \
  -I "$root/apps/esp32/idf/host-test/stubs" \
  "$root/apps/esp32/idf/main/radio.c" "$root/apps/esp32/idf/host-test/radio_endpoint.c" \
  -o "$out/radio-endpoint"
"$out/radio-endpoint"
