# Current status

Updated 2026-10-13 at `main` `fdcb651`. This page is the current summary. The reports it links,
and older pages such as [implementation status](implementation-status.md) and
[software acceptance](software-acceptance.md), are kept as dated history, and their counts are
not current.

**Development software, not flight-qualified.** No board has been flashed. Host, simulator and
compile-only evidence do not establish motor stop timing, target timing, radio behaviour or flight.

## Verification (every push, GitHub Actions)

| Job | What it runs | State |
| --- | --- | --- |
| contracts | `scripts/check.sh`: generation, fmt, Clippy (host and both firmware crates), 95 Rust and 55 Python tests (38 tools, 17 swarm), 400-case legacy scalar oracle (max error 4.77e-7), ABI smoke v1/v2, simulator contracts, swarm harness (7 S1 and 11 S2 scenarios), replay benchmark | pass |
| supply-chain | `scripts/deny.sh`: advisories, licences, bans, sources for all four workspaces | pass |
| kani | 8 model-checked proofs in `crates/flight-core/src/proofs.rs` (`cargo kani -Z stubbing`) | pass |
| fuzz | 6 cargo-fuzz targets, 120 s each | pass |
| gazebo | Plugin smoke, 20 finite scenario runs, paired C++/Rust comparison (`make sim-parity`), three-drone Gazebo swarm flight (`make swarm-gazebo-check`) | pass |
| swarm | Crazyswarm2 (ROS 2 Jazzy) coordinating simulated Flix drones: formation, stop, partial arming; formation with the position loop on the drone (`make swarm-sim`) | pass |

Also verified locally: both ESP32-S3 Xtensa archives build. The full ESP-IDF image link was last
run on 2026-10-05; `idf.py` is not installed on the current development host.

## What changed since the 2026-10-05 reports

| Area | State | Evidence |
| --- | --- | --- |
| Unsafe FFI | Every block has a `// SAFETY:` argument; lint-enforced | [unsafe audit](evidence/unsafe-audit-and-properties.md) |
| Safety policy | Property tests, Kani proofs (mixer, clock, final gate, lifecycle, latched faults, diagnostic window) | [Kani](evidence/kani-proofs.md) |
| Untrusted input | 6 fuzz targets (MAVLink, SBUS, settings, proxy, owner) | [fuzzing](evidence/fuzzing.md) |
| Supply chain and lints | cargo-deny; core denies panics, lossy casts, exact float compares | [supply chain](evidence/supply-chain-and-lints.md) |
| Telemetry | HIGHRES_IMU (full ±2000°/s); due telemetry deferred, not dropped, on a full reply queue | [HIGHRES_IMU](evidence/highres-imu.md) |
| IMU saturation | Detected per axis, reported as STATUSTEXT; Gazebo ABI v2 carries the mask | [saturation](evidence/imu-saturation.md), [ABI v2](evidence/gazebo-abi-v2.md) |
| Fault policy | ADR 003: latched faults survive reset; `SensorInit`; operator fault and denial notices | [ADR 003](adr-003-fault-policy.md) |
| Plant comparison | Root cause found (rigid-contact chatter). Soft contact is the default plant; 20/20 x86 pairs pass, worst 17.5 mm / 0.05° | [plant parity](evidence/plant-parity/README.md) |
| CI | First remote runs exposed 3 pre-existing defects, all fixed; a property test later found the diagnostic-window bug (PR #6) | [CI first runs](evidence/ci-first-runs.md) |
| Licence | Original work Apache-2.0; upstream images excluded (upstream has no licence) | `LICENSE`, `NOTICE` |
| Swarm flight (simulation) | S1 and S2 done in simulation: Crazyswarm2 and Gazebo swarms; position loop in `flight-core` (MAVLink 84/138, heading fusion, `PositionExpired`); 21 `POS_*` parameters, settings record v2 with v1 migration | [swarm spec](../swarm-spec.md), [S1](evidence/swarm-s1/README.md), [S2](evidence/swarm-s2/README.md) |

## Remaining gates (from the [remaining plan](remaining-plan.md))

| Gate | State |
| --- | --- |
| R1 simulator | Comparison resolved and stable. Native multi-model reset and sustained-flight characterisation open |
| R2 client and signing | Software complete; QGroundControl manual flight controls, joystick and real radio untested |
| R3 exact board profile | **Blocked: no board inspected** |
| R4 battery and loss policy | Needs electrical facts and a user decision |
| R5 hardware bench | **Blocked: needs board and instruments** (stop traces, 30-minute timing, NVS power loss) |
| R6 operator flight and release | After R1–R5 |
| R7 extra transports | Proxy software done; RF coexistence and provisioning persistence open |
| Swarm S3/S4 | Positioning hardware (SD-07, UWB proposed) and real swarm flight: **blocked on hardware**. Settings v2 NVS migration not yet exercised on a board |
| ADR 003 D4/D5 | Board-level safe boot (at R3); on-board settings recovery (deferred) |

## Known limits and open decisions

- Simulator evidence before 2026-10-11 used the rigid-contact plant (`FLIX_CONTACT=rigid` reproduces it).
- Full-tick C++/Rust parity cannot pass: Rust applies inputs before control (a safety correction).
  The gate is the stable-interval rule in `fixtures/simulator/acceptance.json`.
- The Gazebo IMU sample lags truth by one 1 ms step in both engines.
- SCALED_IMU still saturates near 1,877°/s for upstream compatibility; HIGHRES_IMU does not.
- Upstream images in `docs/img/` need permission or replacement before any public release.

## Local development on Apple Silicon

Gazebo runs as a native arm64 development image (`.local/gazebo-arm64`, Ubuntu 20.04,
Gazebo 11.15.1) with Colima using `portForwarder: grpc` for UDP. QEMU amd64 Gazebo aborts. CI
remains the x86 evidence source.
