# Gazebo adapter and test profiles

Gazebo 11 applies the Rust owner's final normalized motor values through the
legacy 65 g model. The default estimator has no ground-truth heading correction.
Motor order is RL, RR, FR, FL and coordinates are FLU. The model's documented
motor asymmetry is retained. Nominal scalar parity is separate from full plant
parity and sustained flight.

`make sim-check` builds and exercises the plugin. `make sim-scenarios` runs finite
scenarios in isolated containers with no published browser ports. The existing
browser container is separate. Trace loss fails a measurement; do not treat
unobserved intervals as a stop guarantee.

## Reset

`ModelPlugin::Reset` revokes the owner, its pending commands, frames and transfer
cursors and forgets the learned telemetry endpoint. A separate time-reset event
does the same when Gazebo rewinds time without moving the model. A full reset
can invoke both hooks; successive authority generations are acceptable and
neither hook arms. The trace sequence stays monotonic across all reset forms.
Reset also closes/rebinds the command socket, discarding unread kernel datagrams.
If rebinding fails, control and tracing remain unavailable and the reset owner
has no motor authority. Incoming requests after reset still use the explicit
unsigned loopback development policy.

The external `flix-world-reset` test helper accepts `all`, `time`, or `model`.
The collector launches it asynchronously so a slow transport connection or
shutdown cannot prevent UDP trace collection. Reset kind determines the pose
contract: full/model reset restores origin, whereas time reset preserves position
within the predeclared adjacent-tick tolerance. Virtual IMU transients may latch
a fault. Recovery requires explicit `clear-fault`; no automatic rearm is allowed.
The Gazebo implementation calls model plugin reset from
[Model::Reset](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/Model.cc)
and selects reset variants in
[World::OnWorldControl](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc).

The queued-model case confirms pause through world statistics, queues four
mode/arm/control datagrams, then resets and resumes. It requires cleared authority
and explicit recovery. This exercises the operating-system socket queue as well
as the Rust ingress queue. The helper's `pause` and `model-resume` variants are
test-only orchestration, not flight commands.

## Observation

The optional distinct loopback `trace_port` uses nonblocking fixed packets.
`FLXT` v2 is 192 bytes: v1's 48-byte header, the input's `sample_us` as an
additional little-endian `u64`, then the same 34 floats. The Python decoder
continues to accept 184-byte v1 traces and marks their sample time as assumed.
Reason 11 is the portable Power fault. `FLXC` v1 command traces are unchanged.
SETUP_SIGNING frames are excluded even when rejected, because their payload can
contain a secret.

Normal simulation still assigns input sample time from the tick, matching the
existing adapter. Recording that field does not prove the IMU's real acquisition
time or qualify a physical sensor. It does make an intentionally stale test input
observable without inferring age from a fault reason.

## Explicit sensor fault tests

Default SDF has no injected faults. Isolated acceptance scenarios can opt in with
`test_sensor_fault` set to `invalid` or `stale`, a `test_fault_at_us` timestamp,
and tracing enabled. Missing/invalid configuration fails plugin load. At that
timestamp the adapter changes exactly one input: acceleration X becomes NaN, or
sample time is 10,001 microseconds old. Normal samples resume on the next tick;
the latched fault must keep output stopped. Reset permits the explicitly configured
test to trigger again on its next timeline. Never put these fields in a live
operator world.

The fixtures declare fault time, reason, output-stop tolerance and finite flight
envelope before the run. Invalid raw sensor data is retained as JSON null; the
collector still requires finite pose, estimator, targets and applied motors.

## Two models

The isolated `FLIX_TWO_MODELS=1` runner generates two models separated by 2 m:
command ports 14550/14552 and trace ports 14560/14562. It first arms A while B
remains unselected/stopped, then arms B, stops A while B retains its generation
and flight state, and finally stops B. Every model has its own sequence and stop
latency measurement. The world generator and traces are test tooling; the normal
single-model browser world is unchanged.

## Late client

The adapter always consumes at most four replies per tick and sends them only
when an accepted packet has established an endpoint. Replies produced without
an endpoint are discarded. Holding them would fill the fixed reply slots and
drop the first command ACK on a late connection.

The isolated late-client case starts the repository's actual Python UDP client
after two simulated seconds of telemetry warmup. It checks disarm/mode/arm ACKs,
observes nonzero applied control, disarms, and collects a maintenance timing
snapshot. Its unsigned loopback policy is development-only. It does not qualify
UART, signing over actual RF, or QGroundControl.

## Additional adverse profiles

`make sim-scenarios` includes saturation/recovery, explicit STAB-to-ACRO Stop
and rearm, missed owner invocations with zero applied force, and direct diagnostic
Stop. See [contracts and limitations](../../docs/rust-rewrite/phase6-adverse-integration.md).
The optional `test_gap_at_us` / `test_gap_us` SDF fields require a trace port;
only isolated fixtures set them. During skipped owner calls, lifecycle and
estimator observations remain the previous owner snapshot. The motors in the
trace describe the zero force applied during that interval.
