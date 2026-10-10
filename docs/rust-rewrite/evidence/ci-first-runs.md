# First remote CI runs

Date: 2026-10-10. Repository: `senseix21/flix-rs` (private), branch `hardening/unsafe-and-properties`.
Workflow: `.github/workflows/host.yml` (jobs: contracts, supply-chain, gazebo, fuzz). These are the
repository's first remote runs; before this, CI was "configured but has not run remotely".

| Run | Head | contracts | supply-chain | gazebo | fuzz |
| --- | --- | --- | --- | --- | --- |
| 38024039087 | `ba7f562` | fail: legacy fixture compile (`atol`) | pass | fail: git dubious ownership | pass |
| 38024959087 | `c3e6ee7` | fail: proxy serial write deadline | pass | fail: native-reset notice check | pass |
| 38025743862 | `7a0d395` | fail: proxy serial write deadline | pass | **pass** (all scenarios) | pass |
| 38026624336 | `52b7435` | fail: legacy fixture compile (`isfinite`, libstdc++) | pass | pass | pass |
| 38027769029 | `1287590` | **pass** | pass | **pass** | pass |

## Defects found, all present before this branch

1. **Legacy fixtures did not compile on current compilers.** The pinned `Arduino.h` uses
   `atol`/`atof` without `<cstdlib>`, and `vector.h`/`quaternion.h` call unqualified `isfinite`,
   which libstdc++ declares only in `std::`. Our fixtures now include `<cstdlib>` and `<cmath>`
   and add `using std::isfinite`; the pinned legacy source is unchanged. Verified with Ubuntu
   24.04 clang 18 and g++, and macOS clang.
2. **Gazebo scenarios never started in a container.** git refused the host-owned `/work` bind
   mount (dubious ownership, exit 128), so `release-manifest.py` failed before any scenario.
   `scripts/gazebo-scenario.sh` now marks `/work` safe in the container's git config.
3. **Python serial send starved by the reader thread.** `SerialTransport` shared one non-fair
   lock between `send()` and the reader's tight `receive()` loop, which holds it for up to 10 ms
   per call. `send()` could miss its 0.5 s deadline while the device was writable. Reproduced in
   Ubuntu 24.04 arm64: 9/30 failures at the base commit `dc24b78` and 7/30 on this branch. With
   separate read and write locks: 60/60 pass. A deterministic test holds the read lock and
   requires `send()` within 0.1 s; it fails against a shared-lock copy.
4. **New check too strict (this branch).** The scenario rule "any range-limited tick needs an
   operator notice" failed native reset. The single limit was a 3,600 m/s² impulse 3 ms after the
   world rewind, while the plugin had (by design) forgotten the client endpoint, so all replies
   were discarded. A notice is now required only for limits observed while an endpoint is known.

## Gazebo results (run 38025743862, native amd64)

Every scenario passed. Every scenario with reportable range-limited ticks delivered an
`IMU saturated:` STATUSTEXT, for example landing-contact (9 limited ticks) and mode-change
(11, including `gx`). The four native-reset runs each had one limit inside the reset window and
correctly required no notice.

Finding: almost every scenario saturates all three accelerometer axes once, apparently at spawn or
first ground contact. This is a simulator start-up artifact to investigate separately; it is not
evidence about hardware.

Run 38027769029 is the first fully green run: all four jobs passed, including the proxy serial
round trip and the parity check against the pinned legacy source.

## Local Linux reproduction of the contracts job

`ubuntu:24.04` arm64 under Colima, Rust 1.92.0, clang 18, pinned legacy checkout
`1f5b268…`: `sh scripts/check.sh` exit 0, with 68 Rust tests, 36 Python tests, 400 parity
cases (max error 4.77e-7), ABI smoke v1/v2, simulator contracts and the replay benchmark.
The steps after parity had never run in CI before.
