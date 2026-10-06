# Development completion and remaining acceptance gates

Latest five-item software work and retained failures: [software acceptance](software-acceptance.md).
Latest transport software: [USB / ESP-NOW proxy integration](proxy-integration.md).

Latest simulator extension: [Phase 6 adverse integration](phase6-adverse-integration.md).

Latest update: [reset/metrics integration](reset-metrics-integration.md). The
counts, SDK sizes and provisioning findings in this earlier report describe
the preceding integration pass, not the latest implementation.

Date: 2026-10-05. Repository: `flix-rust`, retained at the user's request.
Status: verified local development delivery; the full rewrite acceptance plan
remains open. Source is uncommitted and has no published release or remote CI run.

## Completed in this integration pass

- Added independent fault replay through the actual simulator C ABI and Python
  wire encoder. Four control intents, nonfinite/stale IMU, lost calibration,
  clock rollback/gaps, diagnostic stop, command loss despite heartbeats and
  rejected traffic, reset with queued authority, independent instances, full
  queue stop preemption and seeded extreme ACRO inputs pass.
- Included those contracts in `scripts/check.sh` and the host CI job.
- Upgraded the actual Gazebo smoke check from connection-only validation to
  ground-start STAB arm, positive motor telemetry, disarm and four zero outputs.
  `make sim-check` builds/runs an isolated container without competing with the
  browser's live simulator. Added a Gazebo CI job.
- Added code-file fingerprints and the firmware lockfile hash to development
  artifact manifests, including untracked code before the first commit.
- Updated the specification's runtime/protocol decisions, stale plan text,
  implementation counts and usage guidance.
- Browser recovery already passes actual Firefox/Gazebo checks: Reset drone
  restores the initial upright world pose, clears fault state, revokes browser
  thrust and leaves the owner disarmed. Camera reset remains separate.

No controller gains, hardware pin assignments or flight-loss policy changed.

## Reproduce

From the repository root, with the pinned legacy checkout at `../flix`:

```sh
make check
make sim-check
rustup target add thumbv7em-none-eabihf --toolchain 1.92.0
cargo check --manifest-path apps/esp32/firmware/Cargo.toml --locked --release --target thumbv7em-none-eabihf
python3 scripts/release-manifest.py target/release/flight-replay --profile host-development
```

`make sim-check` requires Docker. The embedded ARM check verifies allocator-free
portability; exact ESP32-S3 archive and SDK image builds are previous evidence in
[local verification](evidence/local-verification.md), not new target measurements.

The current raw outputs and hashes are in
[evidence](evidence/development-completion.md). Remote CI remains unexecuted.

## Remaining work required by the full plan

| Gate / work | Current disposition | Required next evidence |
| --- | --- | --- |
| Exact board, IMU, pins, power and external gate | Awaiting hardware profile | SKU/markings, schematic and verified wiring |
| Acquisition, scheduling, PWM, boot/stall/panic/reset and workload | Implemented candidate; physically unqualified | Instrumented target traces and 30-minute workload |
| Wi-Fi signing and real QGroundControl interoperability | Actual signed QGC discovery/96-parameter transfer verified locally | Manual UI and physical deployment qualification |
| Persistence, migration and recovery | Host interruption/CRC checks pass | Actual NVS power-interruption, flash and rollback checks |
| Battery and low-voltage policy | Design open; battery values unknown | Electrical profile, calibrated monitor and accepted airframe policy |
| Full simulator acceptance | 20 finite Gazebo scenarios pass | Full C++/Rust plant parity remains failed |
| ESP-NOW and USB proxy | Signed host/proxy integration and provisioning tools verified | Physical radio and profile-specific measurements |
| Physical flight and release | Open | Props-removed gate, operator flight envelope, accepted review and recovery results |

These entries are not closed by the development checks. Optional transport
profiles can be explicitly deferred for a narrower first release; mandatory
requirements for the chosen release still need evidence. The observed browser
flip is recoverable, but recovery does not establish stable or tuned flight.

The next physical step is to complete the hardware profile in
[qualification](qualification.md), then flash with motor power isolated and
measure sensor acquisition and stopped-output behavior. The next simulator
acceptance step is the full plant comparison and adverse-flight scenarios.
