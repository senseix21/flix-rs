# Swarm S2: position control on the drone (simulation)

Date: 2026-10-13. Base: branch `swarm/s2-position`, stacked on `swarm/gazebo-viewer` (`9e42987`).
Requirements: SWM-05 to SWM-10, SD-02, SD-05, SD-06, SD-09, SD-10, SD-11
([swarm spec](../../../swarm-spec.md)). Plan: S2-01 to S2-07. Hardware profile: none.
**Simulation only.**

## In plain terms

The drone now holds and moves between positions itself. The ground sends "you are here"
(position samples) and "go there" (position targets); the Rust flight code does the rest. If
the position samples stop arriving for 200 ms, the drone stops its motors on its own.

## What changed

| Layer | Change |
| --- | --- |
| `flight-core` | New `position` module: flight volume, `Target`, position/velocity `Estimate` (IMU prediction plus external correction), position `Controller` (bounded tilt 25° and thrust 0.9). `Intent::Position`; `Flight::position()` for samples; `Reason::PositionExpired = 13`. `Quaternion::world` and `from_axes` |
| Takeoff rule (SD-10) | Arm from an idle AUTO attitude lease as before; the next command may be a position target. Any other kind change while armed still disarms |
| Loss rule (SD-11) | Flying a position intent with no accepted sample for 200 ms: `Fault`/`PositionExpired`, stopped output |
| `flight-protocol` | Decodes `SET_POSITION_TARGET_LOCAL_NED` (84, LOCAL_NED, masks `0x09C0`/`0x09F8`) and `ATT_POS_MOCAP` (138); NED to NWU at the boundary. Ingress keeps the newest sample (older ones are replays); stop clears it |
| Owner | Applies the newest sample before setpoints each tick |
| Ground (`flix_swarm`) | `Config(onboard=True)`: sends ATT_POS_MOCAP at 50 Hz and position targets at 50 Hz instead of attitudes |

## Commands

```sh
cargo test --workspace --locked
PYTHONPATH=adapters/crazyswarm2 python3 -m flix_swarm.scenarios --onboard --output target/swarm-s2
docker run --rm --init -v "$PWD:/work" -v "$PWD/.local/legacy-git:/legacy:ro" -w /work \
  flix-rust-gazebo:arm64 sh scripts/gazebo-swarm.sh 3 once onboard
```

## Results

**Rust:** 86 tests pass. New: 7 position contract tests (`crates/flight-core/tests/position_contract.rs`)
and 3 wire tests (`crates/flight-protocol/tests/position.rs`). Clippy (host and both firmware
crates), `no_std` checks, fmt and dialect generation pass.

**Tests fail when the rule is removed** (patched locally, not committed):

| Removed | Result |
| --- | --- |
| Freshness check in the position path | `stale_position_stops_the_motors` fails |
| SD-10 takeoff exception and the setpoint volume check | 5 of 7 position contract tests fail |

**Fast harness, on-board loop** (`harness-*.json`, seed 1): all 10 scenarios pass.

| Scenario | Key numbers |
| --- | --- |
| hover (all scenarios) | 18 mm at 4 s after takeoff (the z integral is still absorbing the hover-thrust guess) |
| `formation` | formation errors 7.1 to 10.6 mm; max tilt 4.3°; min pair distance 0.599 m |
| `link-loss` | cf2 stopped 192 ms after the cut with `position-expired` (position limit expires before the 300 ms lease); bystanders within 18 mm |
| `position-dropout` | link up and targets streaming, samples stop: cf2 `Fault`/`position-expired` 212 ms later; returning samples do not revive it |
| `outside-volume` | target at 2.5 m (ceiling 2 m) sent straight to cf1: it did not climb (held 0.483 m, peak 0.486 m) and stopped when its lease ran out, 293 ms later |
| `noisy` | 2 cm position noise, 20 ms latency: hover 16 to 27 mm, formation 11 to 30 mm, max tilt 6.6° |
| `swarm-stop` | every output zero 2 ms after the request |
| `sensor-fault`, `arm-refusal`, `load` | as S1; 8 drones rotate within 22 mm |

**Gazebo** (local arm64 image, Gazebo 11.15.1, 3 drones, `once onboard`): **pass**. Distance from
plan at the end of each step: takeoff 14 mm; zigzag 21 mm; ring 7 to 13 mm; ring turns 8 to 27 mm;
back to the line 23 to 24 mm; landing 5 to 11 mm. All three landed with zero output.

## Not established

- **Heading alignment.** The position loop uses the IMU's yaw estimate; the positioning system's
  yaw is not fused. In simulation both start at zero. On hardware the frames must be aligned or
  the external yaw fused (S3).
- **Tuning is built in.** `PositionTuning::DEFAULT` (gains, volume, 200 ms limit, hover thrust 0.5)
  is not yet operator-configurable; that needs new parameters and a settings schema migration.
- **Not yet proven:** Kani bounds on the position controller; property tests over sample
  sequences; the Crazyswarm2 (ROS) path in on-board mode; CI runs of this branch.
- **Addressing.** `ATT_POS_MOCAP` has no target field. In simulation each drone has its own link;
  on hardware each drone needs its own link or per-drone signing keys (SD-05).
- Nothing here says anything about real positioning accuracy, radio latency or flight.
