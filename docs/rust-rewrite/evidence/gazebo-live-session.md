# Persistent Gazebo session

Date: 2026-10-05. Local uncommitted development source. Gazebo 11.10.2 in the existing `flix-rust-gazebo:local` Docker image, actual Rust archive and C++ plugin. [Changed simulation input hashes](gazebo-live-inputs.json).

Container: `flix-rust-gazebo`. Headless server; no graphical window. The legacy Flix model starts resting on a local ground plane at body center height 0.0138 m. Legacy model mass, geometry, noise and motor-force mapping remain the simulation profile; it is not an electrical model of the proposed development board.

## Commands

From `flix-rust`:

```sh
docker run --init -d --name flix-rust-gazebo \
  -p 127.0.0.1:14550:14551/udp \
  -v "$PWD:/work" -v "$PWD/../flix:/legacy:ro" -w /work \
  flix-rust-gazebo:local sh scripts/gazebo-run.sh
PYTHONPATH=tools python3 tools/gazebo-check.py
```

The plugin remains bound to container loopback. A bounded one-client development UDP relay forwards host loopback traffic, with a 280-byte frame cap and no protocol authority of its own. The Rust owner validates incoming commands. Do not expose the relay as a flight gateway.

## Result

[Client result](gazebo-live-check.json) and [server build/runtime log](gazebo-live-server.txt).

* Parameter 0 returned `0.05000000074505806`.
* STAB mode request and explicit zero-thrust arm were acknowledged.
* Fresh 0.15-thrust simulated inputs produced positive motor telemetry.
* Disarm was acknowledged and subsequent motor telemetry was `[0, 0, 0, 0]`.
* Telemetry included scaled IMU, attitude quaternion, RC, ACK and actuator messages.
* Shell syntax, Python compilation and whitespace checks passed for the changed files.

The first world spawned the model above the ground and the owner denied mode selection in that run. The final world starts the collision body resting on the plane and passes the complete sequence. No fault checks or arming preconditions were removed. State/reason transition logging was added to the plugin for diagnostics. This evidence establishes the final ground-start sequence only; it does not establish the exact cause of the earlier denial or qualify impact recovery.

The container is left running and the client ends disarmed. Restart with `docker start flix-rust-gazebo` after stopping it. The check can be repeated with the same Python command; close other client sessions first. Stop with `docker stop flix-rust-gazebo`.

No target hardware, radio provisioning or flight qualification is asserted. World reset, multiple concurrent model physics, full flight scenarios and instrumented target behavior remain separate gates.
