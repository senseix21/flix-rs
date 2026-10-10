# Unsafe audit and safety-policy properties

Date: 2026-10-10. Base revision: `dc24b78` plus uncommitted working-tree changes.
Requirements: SAF-01, SAF-02, SAF-03, SAF-05, SAF-07, SAF-08; `AGENTS.md` unsafe-boundary rule.
Hardware profile: none. Host and compile-only evidence; no board was flashed.

## Unsafe audit

Before: Clippy `undocumented_unsafe_blocks` reported about 55 unsafe blocks or impls without a
`// SAFETY:` argument across the ESP32 bridge, proxy bridge, Gazebo ABI and tests.

Changes:

- `flix_hal_now`, `flix_hal_stop` and `flix_hal_write` are declared `safe fn` in the
  extern block: they take no pointers, validate their own state in C and lock output access.
- `flix_board_stop`, `flix_board_maintenance` and `flix_board_radio_epoch` became
  `unsafe extern "C"` with a `# Safety` contract. They were safe Rust functions with an
  unchecked owner-task precondition. The C ABI is unchanged.
- Raw copies became bounds-checked slice copies. The proxy output re-checks the 250-byte
  ESP-NOW limit locally instead of relying only on `flight_protocol::proxy::relayable`.
- `flix_board_init`, `flix_board_tick` and `flix_board_metrics` reject misaligned pointers
  (motors stopped / `-1` / `5`) instead of relying on the caller.
- Workspace and both firmware manifests deny `unsafe_op_in_unsafe_fn`,
  `clippy::undocumented_unsafe_blocks` and `clippy::missing_safety_doc`. `scripts/check.sh`
  now runs Clippy on both firmware crates.

Verification: the four Clippy configurations report zero findings. A probe with an
undocumented unsafe block fails the firmware Clippy gate. Both Xtensa archives build with
`cargo +esp rustc ... --target xtensa-esp32s3-espidf -Z build-std=core`. ESP-IDF (`idf.py`)
is not installed on this host, so the SDK image link was not rerun.

## Property test

`crates/flight-core/tests/properties.rs`: 512 cases of up to 300 random operations
(ticks with clock anomalies and bad samples, NaN/infinite/out-of-range intents, source
changes, arm, disarm, faults, maintenance diagnostics, reset, and delayed replay of earlier
frames) under randomized command/frame limits. After each step it checks:

- committed output is finite, in [0, 1], and zero outside Armed/Maintenance;
- maintenance output is at most 0.15 and zero once the diagnostic expires;
- armed output is zero once the selected source's lease is older than `command_us`;
- a replayed frame is zero after `frame_us` or after any generation change;
- Armed is entered only through a successful `arm()` with a fresh command below 0.05 thrust;
- Fault is left only through `clear_fault()` or `reset()`; disarm stops every earlier frame;
- non-finite intents are rejected; an armed mode change is never accepted;
- the generation counter never decreases.

Measured coverage, 100 cases, by output check: Fault 3,935; Standby 3,409; Maintenance 452
(66 non-zero); Armed 330 (271 non-zero); Boot 79.

## Mutation check

Each mutation edited `crates/flight-core/src/safety.rs`, ran
`cargo test -p flight-core --test properties --locked`, then restored the file.

| Mutation | Result |
| --- | --- |
| Accept invalid intents | Killed |
| Diagnostic cap 0.15 → 0.5 | Killed |
| Arm thrust limit 0.05 → 5.0 | Killed |
| Disarm leaves Armed state | Killed |
| `select` clears Fault | Killed |
| Remove commit generation check | Killed |
| Remove commit frame expiry | Killed |
| Remove commit lease check | Killed |
| Remove commit diagnostic expiry | Killed |
| Armed mode change accepted without disarm | Killed |
| Remove commit state check | Survived: tick zeroes values and the generation check rejects older frames |
| Remove commit range check | Survived: the mixer and the 0.15 diagnostic cap never produce out-of-range values |
| Failsafe without invalidation | Survived: commit state check blocks Failsafe |
| Remove tick final zeroing | Survived: commit state check blocks it |
| Tick ignores diagnostic expiry | Survived: commit diagnostic expiry blocks it |

The five survivors are redundant layers: each is backed by another check, so no single
mutation changes committed output. This shows layered checks, not untested contracts.

## Consolidated run

`CXX=<wrapper> FLIX_LEGACY=<pinned checkout> sh scripts/check.sh`: exit 0. Results: 50 Rust tests,
34 Python tests, 400 scalar parity cases (max error 4.77e-7, limit 5e-6), ABI smoke,
simulator contracts and replay benchmark.

## Unresolved

- On this macOS clang the pinned legacy `gazebo/Arduino.h` uses `atol`/`atof` without
  `<cstdlib>`. Parity ran with a compiler wrapper adding `-include cstdlib`; the pinned
  source was not modified. The unmodified tree fails the same way.
- `Flight::reset()` leaves Fault, including `Reason::Configuration`, which `clear_fault()`
  deliberately refuses. Whether this is intended for simulator and owner resets needs a
  recorded decision.
- Proptest seeds are random per run. `crates/flight-core/tests/properties.proptest-regressions`
  holds the 12 shrunk counterexamples found during the mutation runs. Proptest replays them
  first on every run, so those failure scenarios are always rechecked.
