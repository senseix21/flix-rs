# Swarm S2: position control on the drone (simulation)

Date: 2026-10-13. Base: branch `swarm/s2-position` from `main` after #9 (`a0323c3`).
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

## Follow-up (2026-10-13): proofs, heading, parameters, Crazyswarm2

| Item | Change | Evidence |
| --- | --- | --- |
| Proofs | Limiter split into `position::limit`; demand clamped to 100 m/s^2 per axis first (Kani found that a 10^28 m/s^2 demand reached `finite / infinity`; harmless with real IEEE maths, but the clamp removes every overflow path) | Kani `position_limit_bounds_thrust_and_refuses_non_finite` passes in 2.2 s (all 8 harnesses pass with `-Z stubbing`). Property tests: limiter tilt bound with real `libm`; bounded output and stop-on-stale over random sample/target/tick sequences. Removing the freshness check fails the sequence property |
| Heading | `ATT_POS_MOCAP` quaternion heading (FRD/NED to FLU/NWU) corrects the estimate's yaw: first sample in full, then 5 % per sample; a zero or non-unit quaternion means position only; an invalid heading rejects the sample | Core test (full then gradual, roll/pitch untouched, invalid rejected); wire test (NED yaw +0.5 is NWU -0.5; zero/NaN/non-unit give none). Harness `heading`: drones starting at 0.6, 2.0, -2.5 rad turn to the target heading (0.01 rad) and fly the formation within 4 to 16 mm. With heading disabled the same scenario fails (two drones end 6 to 7 m away) |
| Parameters | 21 `POS_*` parameters (volume, freshness, estimator, gains, limits). Settings record version 2: 117 values, 488 bytes. Version 1 (96 values, 404 bytes) still loads with position defaults; the next save writes version 2 to the other slot. NVS read reports the stored length; C accepts exactly 404 or 488; a Rust compile-time assertion pins both sizes | Config tests: v1 migration keeps values and defaults the rest; v1/v2 header and length mismatches and CRC corruption refused; save over a v1 slot writes v2 to the other slot; parameters reach `Tuning::position`; empty volume or hover above max thrust refused as `CrossField`. Firmware Clippy (thumbv7em) passes. **The ESP-IDF image was not built and NVS was not exercised on a board** |
| Crazyswarm2 | `FLIX_POSITION_LOOP=drone` puts `FlixSIL` in on-board mode; `make swarm-sim` adds an on-board formation run | Local arm64: `pass crazyswarm2 formation` with the loop on the drone; hover 21 mm, triangle 9 to 20 mm, line 10 to 15 mm, min pair distance 0.599 m (`crazyswarm2-formation.json`) |

Host benchmark: core state 504 bytes on `main` before S2, 656 after the first S2 commit, 672 now
(heading flag and tuning). Tick time varies 18 to 29 ms per 100,000 ticks on both builds, so no
change is attributable.

## Not established

- Anything on hardware: positioning accuracy and latency, radio, NVS migration on a real ESP32-S3,
  flight. These wait for S3/S4 and the R3/R5 bench work.
- The tilt bound is checked by property tests, not proven by Kani (the proof timed out).
- Lost position stops the motors (SD-11). A flight profile that levels and ramps thrust down
  instead is a separate, later decision.
- `ATT_POS_MOCAP` has no target field: on hardware each drone needs its own link or per-drone
  signing keys (SD-05).
