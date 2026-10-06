# Development integration evidence

Date: 2026-10-05. Local uncommitted source; no remote CI run.
Legacy revision: `1f5b268271536e45a9377d1c5317d3eb6f3c3bba`.
Profile: host development and Gazebo 11 ground-start, no physical hardware.

| Command | Result | Raw output |
| --- | --- | --- |
| `sh scripts/check.sh` | Pass: formatting, Clippy, locked builds, 34 Rust tests, 15 Python tests, 400 C++ nominal cases, independent ABI and 14 simulator replay scenarios | `development-host-check.txt` |
| `make sim-check` | Pass: Docker image/plugin build, parameter, STAB, zero-thrust arm, positive motors, disarm and four zero outputs | `development-gazebo-check.txt` |
| Firmware `cargo check --locked --release --target thumbv7em-none-eabihf` | Pass: allocator-free ARM portability | `development-nostd-check.txt` |
| `python3 scripts/simulator-contracts.py` | Pass: deterministic ABI fault replay, seed 24301 | `development-simulator-contracts.json` |
| `python3 scripts/release-manifest.py target/release/flight-replay --profile host-development` | Pass; repeated source and artifact digests identical | `development-artifact.json` |

The ARM target was initially absent on this host. Installed with
`rustup target add thumbv7em-none-eabihf --toolchain 1.92.0`, then reran successfully.
The ARM check is not an Xtensa build. Earlier exact Xtensa/SDK image evidence is
in `local-verification.md`; those target images were not rebuilt in this pass.

The simulator replay checks actual Rust ABI outputs and bounded Python wire
frames. They do not run Gazebo physics; the independent Docker check does.
Nominal C++ comparison tolerance is 5e-6, maximum absolute error 4.77e-7.
The seeded ACRO stress workload is 1000 ticks. Core benchmark is 100000 armed
STAB ticks and is host timing only, not an embedded deadline result.
Gazebo rendering warnings are expected in the headless container.

The source fingerprint includes tracked and untracked code/build files and
excludes documentation/evidence to prevent self-reference. The manifest labels
the artifact unqualified development. No hardware or flight gate is closed.

Source snapshot SHA-256: `1d457b28a76c5d0dfa4264377788732a2f6641fe91ef2f47157415ef22df8a96`.
Host artifact SHA-256: `e95dbbb4303007ee6eaf997b46616edd4ca201981cde8bfc6845fac1d5dd8f65`.
The live browser simulator remains upright, disarmed and at zero motor output.
See `completion-report.md` one directory above for unresolved acceptance gates.
