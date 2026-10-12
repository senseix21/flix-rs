# Crazyswarm2 adapter: simulated Flix swarm (S1)

Several simulated Flix drones, each running the real `flight_esp32::Owner` around
`flight_core::Flight` through the `flight-gazebo` C ABI, coordinated as a swarm.
Requirements: [swarm specification](../../docs/swarm-spec.md). Work items: [swarm plan](../../docs/swarm-plan.md).

**Simulation only.** This shows that coordination works with Flix's real control and safety
code. It says nothing about hardware, positioning or on-board position control.

## In plain terms

Each drone in the simulator runs the same flight code as the real board. A ground program
works out, for each drone, "tilt this way, this much thrust" to reach the swarm's next
position, and sends it as an ordinary Flix command. The drone checks and applies it exactly as
it would on hardware, and stops on its own if the commands stop arriving.

## Two ways to run it

| Path | What runs | Command | Needs |
| --- | --- | --- | --- |
| Fast harness | `flix_swarm` package: Flix owners, Flix plant, a minimum-jerk planner | `PYTHONPATH=adapters/crazyswarm2 python3 -m flix_swarm.scenarios --output DIR` | `cargo build --release -p flight-gazebo`, Python 3.9+ |
| Crazyswarm2 | Crazyswarm2's simulator server, firmware planner and `crazyflie_py` scripts, with `FlixSIL` and the `flix` backend | `make swarm-sim` | Docker |

Both use the same `Vehicle` (owner, IMU synthesis, bounded link, ground position loop) and the
same `Plant`. `scripts/check.sh` runs the fast harness and its unit tests on every push; the
`swarm` CI job runs the Crazyswarm2 path.

## How a command reaches the motors

```
Crazyswarm2 takeoff/goTo/land   (ROS service)
  -> Crazyflie firmware planner  (position, velocity, acceleration, yaw)
  -> ground position loop        (flix_swarm/position.py; bounded tilt and thrust)
  -> SET_ATTITUDE_TARGET         (MAVLink, addressed to this drone's system ID)
  -> flix_receive -> Owner       (validation, AUTO lease, arming state)
  -> flix_tick2  -> final gate   (motor outputs, RL RR FR FL)
  -> flix backend / Plant        (forces and torques from adapters/gazebo/plugin.cpp constants)
```

- **System IDs.** Every owner starts as system 1. The vehicle provisions it through the operator
  path (`maintenance`, `PARAM_SET SF_RC_LOSS_TIME`, `PARAM_SET MAV_SYS_ID`, `exit`) and waits for
  both `PARAM_VALUE` confirmations. No ABI change was needed.
- **Arming.** Crazyswarm2's simulator had no arm service; the patch adds `cfN/arm` and `all/arm`.
  Arming sends AUTO mode, a zero-thrust setpoint and `ARM`, one frame per tick, and waits for the
  ACK. Takeoff, goTo and land are ignored for a drone that is not armed.
- **Emergency.** `all/emergency` (previously "not yet implemented" in the simulator) sends each
  drone its own `COMMAND_LONG 400` stop, which the owner routes to its reserved stop path.
- **Lease.** Setpoints stream at 50 Hz. The command lease is provisioned to 0.3 s, so a drone that
  stops hearing the ground drops to Failsafe with zero output within 0.3 s.
- **Separation.** Plans are checked before any drone moves: synchronized straight-line paths must
  stay 0.3 m apart. There is no drone-to-drone avoidance.
- **Ground truth.** The position loop reads simulator truth in place of a positioning system.

## Files

| File | Role |
| --- | --- |
| `flix_swarm/owner.py` | ctypes binding to `adapters/gazebo/flix.h`; MAVLink frames via `tools/pyflix` |
| `flix_swarm/plant.py` | Rigid body, four motors, soft four-corner ground contact |
| `flix_swarm/imu.py` | Gyro and specific force with legacy SDF noise and MPU6050 range limits |
| `flix_swarm/vehicle.py` | One drone as the ground sees it: link, provisioning, arming, streaming, loss detection |
| `flix_swarm/position.py` | Ground position loop (S1 only; SD-02) |
| `flix_swarm/swarm.py`, `planner.py` | Fast harness clock, commands and plan checks |
| `flix_swarm/scenarios.py` | S1-09 scenarios and thresholds |
| `ros/flix_sil.py` | `CrazyflieSIL` subclass that drives a `Vehicle` |
| `ros/flix_backend.py` | `crazyflie_sim` backend stepping `Plant` at 1 kHz |
| `ros/crazyflie_server.patch` | Adds the `flix` controller, arm services and a working emergency stop |
| `ros/demo.py`, `ros/check.py` | Coordinator scripts on stock `crazyflie_py`, and their trace checks |

## Plant fidelity

The plant uses the Gazebo model's mass, inertia, motor constants, motor asymmetry and soft
contact profile, but it is a separate integrator (semi-implicit Euler, 0.5 ms substeps) and has
no motor lag or drag. It is not a substitute for the Gazebo comparison evidence.
