# Local run of the CI gates, 2026-10-16

GitHub Actions has been on hold since 2026-10-13 (spending limit), so these CI jobs were run
locally on the PR #11 branch at revision `b8414e4`. Host: Apple Silicon (arm64) macOS; Rust
1.92.0 stable, nightly 1.101.0 (`1d81eb4ad`, 2026-10-07) for fuzzing. Tool versions match the
CI pins. Requirements: the R2 client and signing work in PR #11, ADR 003 (Kani D1), the
fuzzing and supply-chain gates. No hardware.

| CI job | Command (as in `.github/workflows`) | Result |
| --- | --- | --- |
| contracts | `FLIX_LEGACY=$PWD/.local/legacy-git sh scripts/check.sh` | pass: 110 Rust tests, 38 tools tests, 18 swarm tests, 400 parity cases (max error 4.77e-7), 0 warnings |
| supply-chain | `sh scripts/deny.sh` (cargo-deny 0.20.2) | pass: advisories, bans, licences and sources OK for all 4 workspaces (advisory database fetched on the day) |
| kani | `cargo kani -Z stubbing` in `crates/flight-core` (Kani 0.68.0, CBMC 6.11.0) | pass: 8 of 8 harnesses verified, 0 failures |
| fuzz | `sh scripts/fuzz.sh 120` (cargo-fuzz, 120 s per target) | pass: no crash, no artifact |

Fuzz runs (120 s each, existing corpora and seeds):

| Target | Runs |
| --- | --- |
| `config_record` | 12,346,843 |
| `owner` | 638,092 |
| `proxy_bridge` | 3,485,441 |
| `sbus` | 1,382,970 |
| `wire_decode` | 10,492,279 |
| `wire_valid` | 18,946,112 |

Not run locally: the `gazebo` and `swarm` jobs in their amd64 CI image. The Gazebo plugin was
exercised the same day in the local arm64 image by the QGroundControl operator session
([r2-clients](r2-clients/README.md)); `scripts/check.sh` includes the swarm S1/S2 scenarios.

A first Kani attempt at the workspace root failed before verification (`proxy-bench` needs the
`host` feature); CI runs Kani in `crates/flight-core`, and the rerun there is the result above.
