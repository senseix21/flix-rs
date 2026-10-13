# Swarm S1: Crazyswarm2 coordinating simulated Flix drones

Date: 2026-10-13. Base: branch `swarm/spec` from `main` `ccf1761`, swarm commit `a2137b4` plus the
S1 changes committed with this record. Requirements: SWM-01, SWM-02, SWM-03, SWM-04, SWM-11,
SWM-12, SWM-13 ([swarm spec](../../../swarm-spec.md)). Plan: S1-01 to S1-10.
Hardware profile: none. **Simulation only.**

## In plain terms

Three simulated Flix drones took off, flew a formation and landed on Crazyswarm2's commands. Each
one ran the real Flix flight code. Losing one drone's link, sensor or arming affected only that
drone. A swarm-wide stop zeroed every drone's motors within one or two control ticks.

## What ran

| Item | Value |
| --- | --- |
| Flight code | `flight-gazebo` release library: `flight_esp32::Owner` around `flight_core::Flight`, one per drone |
| Command path | MAVLink 2 frames into `flix_receive`, bench policy (system 255, component 190); no ABI change |
| Plant | `adapters/crazyswarm2/flix_swarm/plant.py`: Gazebo model mass and inertia, plugin motor constants and asymmetry, soft contact |
| Control rate | `flix_tick2` every 1 ms of simulated time |
| Lease | `SF_RC_LOSS_TIME` provisioned to 0.3 s; setpoints at 50 Hz |
| Crazyswarm2 | `f1e09954f56cd08e7d9aff5e13dc9a68795914a3` + `adapters/crazyswarm2/ros/crazyflie_server.patch` |
| Crazyflie firmware (planner bindings) | `2025.02` |
| Image | `flix-crazyswarm2:local`, ROS 2 Jazzy, Python 3.12.3, arm64 (Colima, Apple Silicon), `sha256:7ac88433…` |
| Fast harness host | macOS arm64, Python 3.9.6, no ROS |

## Commands

```sh
cargo build --release --locked -p flight-gazebo
PYTHONPATH=adapters/crazyswarm2 python3 -m unittest discover -s adapters/crazyswarm2/tests -v
PYTHONPATH=adapters/crazyswarm2 python3 -m flix_swarm.scenarios --output target/swarm-s1
make swarm-sim    # Docker: formation, stop, partial-arm through Crazyswarm2
```

## Results

Unit tests: 15 pass (geometry, plant, planning, link bounds, registry bound, addressing, explicit arming).

### Fast harness (`harness-*.json`, seed 1, zero link latency)

| Scenario | Drones | Result | Key numbers |
| --- | --- | --- | --- |
| `single` | 1 | pass | hover error 7.9 mm; `go_to` error 22.1 mm at plan end; max tilt 7.8° |
| `formation` | 3 | pass | crossing plan rejected before anyone moved; formation errors 6.7 to 8.7 mm; max tilt 3.4°; min pair distance 0.599 m |
| `swarm-stop` | 3 | pass | each output zero 2 ms after the stop request (frame sent next tick, applied the tick after); none re-armed in 1 s |
| `link-loss` | 3 | pass | cf2 Failsafe/CommandExpired 292 ms after the cut (limit 325 ms); ground marks it lost on heartbeat timeout 1.5 s later; 25 late setpoints after restore do not revive it; bystanders within 8.1 mm |
| `sensor-fault` | 3 | pass | NaN acceleration: cf3 Fault/InvalidNumeric on the same tick; bystanders within 7.9 mm |
| `arm-refusal` | 3 | pass | cf2 uncalibrated: arm denied with reason `calibration`; cf2 motors stayed 0 through takeoff; cf1 and cf3 flew |
| `load` | 8 | pass | ninth owner refused; 8 drones rotate one position around a 1 m circle; 50 setpoint frames/s each, 400 total; no frames dropped |

### Crazyswarm2 (`crazyswarm2-*.json`)

Stock `crazyflie_py` calls (`arm`, `takeoff`, `goTo`, `land`, `emergency`) from `ros/demo.py`;
the firmware planner produces targets.

| Scenario | Result | Key numbers |
| --- | --- | --- |
| `formation` | pass | system IDs 11, 12, 13; hover 8.1 to 8.2 mm; triangle 4.1 to 11.5 mm; line 5.8 to 6.9 mm; crossing rejected; min pair distance 0.598 m; all landed with zero output |
| `stop` | pass | `all/emergency`: every output zero 1 ms after the request |
| `partial-arm` | pass | only cf1 and cf3 armed; the server ignored takeoff and land for cf2 (`standby, not armed`); cf2 never armed and never moved |

The Crazyswarm2 stop latency (1 ms) is shorter than the harness's (2 ms) because the ROS service
call is handled between ticks, so the stop frame goes out with the next tick's link step.

### Gazebo (`make swarm-gazebo-check`)

Three Flix models in Gazebo 11.15.1 (local arm64 image `flix-rust-gazebo:arm64`, soft contact),
each with its own Rust plugin, flown by `flix_swarm.gazebo` over UDP with plugin traces as
position truth. One flight on 2026-10-13: **pass**. Distance from plan at the end of each step
(each step leaves at least 1 s after its move ends):

| Step ends | flix1 | flix2 | flix3 |
| --- | --- | --- | --- |
| Takeoff | 14 mm | 14 mm | 14 mm |
| Zigzag | 8 mm | 8 mm | 8 mm |
| Ring | 5 mm | 11 mm | 5 mm |
| Ring turn 1, 2, 3 | 6 / 4 / 5 mm | 13 / 4 / 5 mm | 16 / 5 / 4 mm |
| Back to the line | 4 mm | 4 mm | 4 mm |
| Landing | 2 mm | 8 mm | 7 mm |

All three then stopped on the ground with zero output. An earlier version of the flight plan
went straight from a once-rotated ring back to the line; the separation check rejected it
before any drone moved, so the plan now completes a full turn first. With 0.5 s of settling the
ring turns ended 44 to 47 mm behind the moving plan; the turns now wait 1.5 s.

### Checks fail when safety paths are removed

Run on the fast harness by patching the harness at runtime (not committed):

| Mutation | Scenario | Result |
| --- | --- | --- |
| Stop request recorded but never sent | `swarm-stop` | fails |
| Link cut does nothing (ground keeps streaming) | `link-loss` | fails |
| Plan separation check always passes | `formation` | fails |

## Observations

- Landings and the fault drop produce `IMU saturated: az` notices: touchdown on the stiff
  contact exceeds the ±4 g register range, as in the Gazebo landing run
  ([IMU saturation](../imu-saturation.md)). Control is unaffected; they are recorded in the results.
- The harness and the `flix` backend use their own integrator, not Gazebo. Tracking errors here are
  not comparable with Gazebo plant-parity numbers.
- The ground loop reads simulator truth. Position noise, delay and dropouts are S2 work.

## Not established

- Anything about hardware, radio links, real timing or real positioning (S3, S4).
- On-board position control (S2). The position loop here runs on the ground and is S1-only.
- The Crazyswarm2 numbers above are from the local arm64 run. The `swarm` CI job (amd64, same
  Dockerfile) passed on PR #8 in GitHub Actions runs 38296795420 and 38296803293 on 2026-10-13.
- `scripts/check.sh` stops at `scripts/legacy-parity.py` on the dev host because the legacy checkout
  `../flix` is missing locally (unrelated to this change). The `contracts` CI job, which has it, ran
  the full script including the swarm steps and passed on PR #8.

## Later fix: arming when frames arrive in bursts (2026-10-13)

The Gazebo CI job failed once (run 38419153537, push of `e7cc096`) while the PR run of the same
commit passed. All three drones refused to arm with `no command`. Under load the plugin handed
mode, zero-thrust lease and arm to the owner in one tick. The owner applies setpoints before
queued commands, and selecting a source clears every lease, so the lease was lost. This is
fail-safe: the drones stayed disarmed. A burst test also found a second race. `exit`,
addressed to the new system ID, can land in the same tick as the `MAV_SYS_ID` change, before
the change takes effect. It is then dropped and the drone stays in maintenance.

The coordinator (`Ground`) now waits for the drone's reply at each step. It sends `exit` only
after both parameter echoes and ends provisioning on its `> exit` / `ok` reply. It sends the
lease and arm only after the mode ACK. The firmware is unchanged. The new test
`test_arms_when_the_transport_delivers_frames_in_bursts` delivers frames in 20 ms bursts. It
fails on the old coordinator (`mode denied`) and passes now. The S1/S2 suites and a local
three-drone Gazebo run pass.
