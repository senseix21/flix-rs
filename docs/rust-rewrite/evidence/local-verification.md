# Local verification evidence

Date: 2026-10-05. Source: local **uncommitted** `flix-rust` development tree. Host: Intel macOS. Legacy C++ revision: `1f5b268271536e45a9377d1c5317d3eb6f3c3bba`. No physical board was attached, flashed or operated. No flight-qualified profile exists.

[Source-input SHA-256 inventory](source-inputs.json) identifies the delivered implementation inputs. [Artifact hashes](artifacts.json) identify locally built images; paths in that record are temporary local build outputs, not published binaries. Raw logs retain staging paths where applicable. The target SDK is ESP-IDF v5.4.2, commit `f5c3654a1c2d2a01f7f67def7a0dc48e691f63c0`; the target compiler is Espressif Rust v1.88.0.0 and SDK Xtensa GCC 14.2_20241119. Host Rust is 1.92.0.

## Results

| Check | Workload/threshold | Result and raw evidence |
| --- | --- | --- |
| Generated inventory, formatting, Clippy | Frozen schema/dialect match; no formatting drift or Clippy warnings | Pass, [host check](host-check.txt) |
| Rust contracts | 34 tests: transitions, authority/expiry, maintenance, numerical properties, malformed and valid-CRC packet fuzz, signing/replay, RC, IMU bus, storage, allocation and ABI | 34 passed, [host check](host-check.txt) |
| Python client/log contracts | Immediate ACK, callback isolation, close wakeup, duplicate requests, signed-network policy, actual Rust UDP owner, log offsets/completeness | 7 passed, [host check](host-check.txt) |
| Nominal controller parity | 400 deterministic cases, seed `24301`, absolute tolerance `5e-6`, actual pinned C++ headers | Pass; maximum absolute error `4.769999999432173e-7`, [host check](host-check.txt) |
| Legacy defect probes | Invalid torque disarm, stale AUTO/control timestamps and clock rollback | Legacy failures reproduced; corrected contracts covered in Rust, [host check](host-check.txt) |
| Independent C ABI | Fixed 48-byte input/output layouts, stopped output, stale handles | Pass, [host check](host-check.txt) |
| Embedded portability | ARM `thumbv7em-none-eabihf`, four portable packages, release/locked | Pass, [no_std check](no-std-check.txt) |
| Actual target Rust | `xtensa-esp32s3-espidf`, `build-std=core`, locked release, aborting panic | Pass, [Xtensa build](xtensa-build.txt) |
| Actual SDK link: default | Unwired pins, PWM disabled; no Wi-Fi provisioning | Pass, [SDK build](sdk-unwired-build.txt) |
| Actual SDK link: enabled branches | Compile-only PWM, I²C, RC and Wi-Fi configuration; never flashed | Pass, [SDK build](sdk-enabled-build.txt); 0xf16e0 bytes, 6% of 1 MiB app partition free |
| Actual Gazebo plugin | Ubuntu 22.04 Docker, Gazebo 11.10.2, Linux Rust archive and C++ shared plugin; real world loads, parameter response and disarm ACK | Pass, [Gazebo build/smoke](gazebo-build-smoke.txt) |
| Host benchmark | 100,000 armed STAB ticks with fresh sensor/setpoint and final commit | 82,967 µs total; `Flight` 496 bytes and `Owner` 66,816 bytes, [host check](host-check.txt); **not target timing** |

The state/packet properties use fixed deterministic workloads, including 20,000 mixer cases, 10,000 transition steps and mutated/valid-CRC packet inputs. Storage tests interrupt each of the 404 record-byte write boundaries and corrupt each byte. The allocation test measures armed control ticks rather than only disarmed execution. Consult the tests for the exact seeded generator and assertions.

## Reproduction commands

From the delivered `flix-rust` repository:

```sh
sh scripts/check.sh
cargo +stable check -p flight-core -p flight-protocol -p flight-config -p flight-esp32 --locked --release --target thumbv7em-none-eabihf
```

The recorded ARM `stable` alias was Rust 1.92.0 with that target installed. The host consolidated command was executed directly in the delivered sibling repository with the actual legacy checkout at `../flix`.

From the staging tree, using the installed target compiler:

```sh
. /private/tmp/flix-esp-export.sh
cargo +esp rustc --manifest-path apps/esp32/firmware/Cargo.toml --locked --release --target xtensa-esp32s3-espidf -Z build-std=core -- -C panic=abort
export IDF_TOOLS_PATH=/private/tmp/flix-idf-tools
. /private/tmp/flix-esp-idf-v5.4.2/export.sh
idf.py -C apps/esp32/idf build
idf.py -C apps/esp32/idf -B /private/tmp/flix-idf-enabled-build -D SDKCONFIG=/private/tmp/flix-enabled-sdkconfig -D SDKCONFIG_DEFAULTS=/private/tmp/flix-compile-profile.defaults build
```

[Compile-only configuration](compile-only-profile.defaults) exercises code branches with arbitrary test pins and placeholder Wi-Fi credentials. It is **not** an approved wiring profile. These commands only compile/link; no flash command was executed.

Gazebo was built and run with:

```sh
docker build -t flix-rust-gazebo:local -f adapters/gazebo/Dockerfile .
docker run --rm -v /private/tmp/flix-rust-implementation:/work -v /Users/abuhamzah/Dev/erik/flix:/legacy:ro -w /work flix-rust-gazebo:local sh scripts/gazebo-smoke.sh
```

The image built successfully. Gazebo reports an unavailable audio device in the headless container; parameter and disarm checks still pass. The smoke test does not demonstrate physics/control parity or flight. No remote CI run is claimed.

## Corrections made during verification

* The actual C++ oracle exposed a missing ACRO yaw dead zone; restoring the original dead zone brought all nominal cases within tolerance.
* Restricted sandbox sockets prevented the first UDP test attempt; rerunning with local sockets permitted established the transport contract.
* A fixed 120 ms test sleep was insufficient under concurrent SDK/container compilation. The final test renews its setpoint and waits for observed positive motor telemetry, then waits for zero output after disarm, with bounded two-second deadlines.
* The staging parity command initially lacked its legacy checkout path. The final consolidated command ran from the delivered repository and resolved `../flix` normally.
* Target build checks corrected the static archive path and moved the large startup log into static storage. Both target profiles subsequently linked.
* Missing NVS records are distinguished from corrupted records and read failures; only truly absent records initialize first-boot settings. Failed/corrupted reads cannot silently authorize writes of replacement defaults.

## Open evidence

SAF-06 has no measured physical stop bound. SAF-01/02/03/04/05/07/08 have software evidence and still require the board-specific traces described in [qualification](../qualification.md). Actual hardware identity/calibration, output voltage/frequency/polarity, stall/reset/panic, sample-to-output latency, 30-minute loaded timing, stack/heap high-water marks, radio signing/provisioning/reconnect, RC/ESP-NOW coexistence, battery/low-voltage behavior, recovery flashing and operator flight are unverified.

The 64 KiB log explains most of `Owner`'s size and is statically stored on the target. Host size measurements do not establish target stack usage. Enabled-image partition headroom is limited and must be included in release planning. Optional ESP-NOW USB proxy/serial-client integration and full simulator qualification remain implementation work, not hardware evidence that can be inferred from compilation.
