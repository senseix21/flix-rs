# Flix Rust

Rust rewrite of the Flix ESP32 quadcopter controller. The repository contains a deterministic `no_std` control core, protocol/configuration services, an ESP32-S3/MPU6050 firmware candidate, host replay and a Gazebo adapter.

**Development status:** software implementation spans Phases 0–8, with additional wired radio enrollment, authenticated replies, an opt-in battery guard and timing counters. Physical bench, battery acquisition, transport qualification and flight qualification remain open. PWM defaults to disabled and all board pins are unwired. Read the [execution report](docs/rust-rewrite/execution-report.md), [remaining plan](docs/rust-rewrite/remaining-plan.md) and [development runbook](docs/rust-rewrite/development-runbook.md) before using a target artifact.

## How it works

Validated sensor samples and fresh authorized commands enter one Rust owner. The owner updates attitude/rates, selects STAB/ACRO/AUTO control, computes torque, and mixes four motor commands in rear-left/rear-right/front-right/front-left order. One final gate checks lifecycle, source lease, sensor age, generation and frame expiry. Disarm and faults stop output and invalidate cached frames. The SDK boundary adds a separate expiring physical motor-enable gate; its behavior still requires instrumented measurements.

| Package | Responsibility |
| --- | --- |
| `flight-core` | Time, FLU math, estimator, PID/mixing, lifecycle/safety, SBUS and log ring |
| `flight-protocol` | Bounded MAVLink 2 subset, signing/replay validation, ingress, telemetry and peer limits |
| `flight-config` | Generated 96-name schema, validation, versioned CRC records and interrupted-write recovery |
| `flight-esp32` | Single-owner integration and bounded MPU6050 register driver |
| `flight-replay` | CSV scenarios, legacy comparison, host benchmark, loopback bench and settings tools |
| `flight-gazebo` | Owned instances behind a versioned C ABI; Gazebo 11 plant adapter |

## Run on the host

```sh
cargo run --release -p flight-replay --bin flight-replay -- --demo
cargo run --release -p flight-replay --bin flight-replay -- --benchmark
sh scripts/check.sh
```

The consolidated check includes Clippy, Rust tests, Python client tests against a real loopback Rust owner, C++ parity, ABI checks and deterministic simulator fault replay. Run `make sim-check` for actual Gazebo arm/motor/disarm checks in an isolated container. Keep the pinned legacy checkout at `../flix`, or set `FLIX_LEGACY`. A restricted environment must permit loopback sockets for the client test. Embedded portability can additionally be checked with:

```sh
rustup target add thumbv7em-none-eabihf
cargo check --manifest-path apps/esp32/firmware/Cargo.toml --locked --release --target thumbv7em-none-eabihf
```

The ARM check establishes `no_std` portability, not an ESP32 build.

## Firmware and simulator

The target build uses Rust Xtensa v1.88.0.0 and ESP-IDF v5.4.2. Install the named toolchain with espup, source its export file and ESP-IDF's `export.sh`, then run `sh scripts/build-esp32.sh`. Configure an actual verified bench profile with `idf.py -C apps/esp32/idf menuconfig`; default output is disabled. The first profile is a development candidate; follow the [qualification/recovery procedure](docs/rust-rewrite/qualification.md).

For Gazebo 11, build the host Rust archive, then `cmake -S adapters/gazebo -B target/gazebo` and `cmake --build target/gazebo`. The supplied Dockerfile supports Linux verification. Point a copy of the legacy model's plugin at `libflix_rust.so`, assign a distinct loopback UDP port per instance, and preserve the legacy plant assets. Heading truth is not injected into ordinary scenarios.

## Documentation

- [Completion report and remaining gates](docs/rust-rewrite/completion-report.md)
- [Remaining implementation and qualification plan](docs/rust-rewrite/remaining-plan.md)
- [Architecture](docs/rust-rewrite/architecture.md)
- [Specification](docs/rust-rewrite-spec.md) and [phase plan](docs/rust-rewrite-plan.md)
- [Behavior/compatibility changes](docs/rust-rewrite/compatibility.md)
- [Runtime decision](docs/rust-rewrite/adr-001-runtime.md)
- [Protocol/authentication contract](docs/rust-rewrite/protocol.md)
- [Legacy architecture review](docs/architecture-review.md) and [source provenance](docs/legacy-source.md)

The engineering approach follows this repository's [AGENTS.md](AGENTS.md). It draws on Jon Gjengset's teaching habits without attributing this implementation to him or to Helsing.

## Gazebo smoke test

With Docker and the pinned legacy checkout at `../flix`:

```sh
docker build -t flix-rust-gazebo:local -f adapters/gazebo/Dockerfile .
docker run --rm -v "$PWD:/work" -v "$PWD/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-smoke.sh
```

This compiles and loads the actual plugin with the ground-start world, then checks a parameter response, STAB mode, explicit arm, positive motor telemetry and stopped outputs after disarm. It does not establish sustained flight or physics parity.

## Persistent Gazebo simulation on macOS

The Docker server runs headlessly with the Flix model resting on a ground plane.
The model retains the legacy mass and motor geometry; it does not model a specific
ESP32 development board or qualify its electrical hardware.

```sh
docker run --init -d --name flix-rust-gazebo \
  -p 127.0.0.1:14550:14551/udp \
  -v "$PWD:/work" -v "$PWD/../flix:/legacy:ro" -w /work \
  flix-rust-gazebo:local sh scripts/gazebo-run.sh
PYTHONPATH=tools python3 tools/gazebo-check.py
docker logs --tail 30 flix-rust-gazebo
```

The check connects from the host, reads a parameter, selects STAB, arms with zero
thrust, observes low-power simulated motor telemetry, disarms and verifies zero
motor telemetry. A bounded UDP relay connects the container's loopback-only
plugin to host loopback. It serves one development client at a time and is not a
flight gateway. Close other clients before running the check.

Stop with `docker stop flix-rust-gazebo`; resume with
`docker start flix-rust-gazebo`. There is no graphical window in this headless
configuration. The simulation's default outputs are disarmed.

## Browser viewer and keyboard control

With `flix-rust-gazebo` running, start the browser viewer from the repository:

```sh
python3 tools/gazebo-viewer.py
```

Open **http://127.0.0.1:8765**. Drag to orbit, scroll to zoom, or use the camera
buttons. The stylized drone uses the actual Gazebo model pose; motor/state data
comes from passively observed Rust replies. The viewer has no external assets or dependencies. Its simulation controls require
explicit arming and fresh browser input; browser input loss stops control. The viewer connects a local simulation client for controls and telemetry.

For terminal control, start the viewer with `python3 tools/gazebo-viewer.py --view-only` so its browser client does not compete with the terminal:

```sh
cd /Users/abuhamzah/Dev/erik/flix-rust
PYTHONPATH=tools python3 tools/gazebo-control.py
```

| Key | Simulation control |
| --- | --- |
| A | Arm with zero thrust |
| Space | Disarm and reset controls |
| W / S | Increase / decrease thrust by 2% |
| I / K | Pitch |
| J / L | Roll |
| Q / E | Yaw |
| C | Disarm and request explicit latched-fault clear |
| X / Ctrl+C | Exit and disarm |

Attitude/yaw inputs recenter after 150 ms unless repeated. The client sends fresh
setpoints while running. Use one control client at a time; do not run the automated
check concurrently. Keyboard control is restricted to the local simulation.
The simulator enforces its normal source, arming, fault and expiry rules.

### Flight controls in the browser

Open http://127.0.0.1:8765/?v=5. Press **Arm**, gradually increase **Thrust**,
then hold the direction or yaw buttons. **Disarm** stops output. Use WASD for
tilt, Q/E for yaw, arrow up/down for thrust, and Space to disarm. The viewer
must remain focused and visible; changing tabs/windows requests disarm. Input
loss expires the browser control lease after 500 ms and requests disarm.

If the drone flips or crashes, press **Reset drone**. This stops control and
restarts the local Gazebo container, restoring the drone upright at its initial
position and resetting controller faults. Wait for **Ready**, then arm explicitly.
It resets the whole simulated world; **Reset view** changes only the camera.

This profile controls only the localhost Gazebo simulation. The HTTP control
API requires the viewer session token and matching localhost Origin. Do not run
another flight client at the same time. The separate Rust owner continues to
validate arming, freshness, sensor state and final motor authority. No hardware
qualification is inferred from browser controls.

## Simulator acceptance probes

Run `make sim-scenarios` for isolated, finite level-thrust and pitch-pulse physics
probes. Inputs and thresholds are declared in `fixtures/simulator/acceptance.json`.
Raw per-tick JSONL, command observations, results and logs go to
`.local/qualification/`. The viewer's live container is unaffected.

Tracing is opt-in through the model SDF `trace_port`; default simulation behavior
and gains are unchanged. The collector writes files outside the controller.
Dropped packets invalidate timing conclusions. These finite probes do not
qualify sustained flight. Read the [acceptance matrix](docs/rust-rewrite/acceptance-matrix.md).

The optional [provisioned serial / ESP-NOW proxy](apps/proxy/README.md) now has
a separate Rust application and signed serial host integration. See its
[verification report](docs/rust-rewrite/proxy-integration.md); physical radio
qualification remains open.
