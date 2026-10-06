# Remaining acceptance matrix

Latest five-item software work and retained failures: [software acceptance](software-acceptance.md).
Current implementation and per-scenario results: [reset/metrics integration](reset-metrics-integration.md).
This first inventory is retained as the baseline; its awaiting software entries
are superseded where the execution report records a delivered slice. Overall
hardware, actual-client, full physics parity and operator gates remain open.

Date: 2026-10-05. First R0 slice. Status values mean pass, fail, awaiting evidence,
or awaiting a named prerequisite; they are not completion percentages.

The original requirements and SAF invariants remain authoritative. The user has
authorized execution of the remaining plan; optional release-scope proposals
are still decisions to record, not authorization to silently remove features.

| Work | Original gate / safety requirement | Observable contract | Evidence/check | Current evidence |
| --- | --- | --- | --- | --- |
| R0: manifest and contract | P0-06, P5-06, P8-06 | Inputs, units, timings, thresholds, workload and hashes declared before result | `fixtures/simulator/acceptance.json`, source/artifact manifest | Finite Gazebo probes declared; full hardware workload awaiting profile |
| R1: nominal controller | P2-05, SAF-08 | Scalar nominal error ≤5e-6 | `scripts/legacy-parity.py`: 400 cases, seed 24301 | Pass; not physics parity |
| R1: arm and lifecycle stop | P6-05, SAF-01/02/05/08 | Low-thrust explicit arm; all stopped states zero; received stop applied within one 1 ms tick | Plugin trace and `tools/gazebo-scenario.py` | Rust finite probes pass; legacy one-tick disarm finding retained |
| R1: plant attitude | P6-02, SAF-08 | Finite actual pose; ≤45° tilt during declared short nominal inputs; lift ≥0.015 m; no commanded-flight fault | Level-thrust and pitch-pulse fixtures | Two Rust probes pass; full paired motor comparison fails; sustained flight open |
| R1: IMU/clock/leases | P6-05, SAF-03/08 | Fault/expired authority produces four zero outputs; no implicit rearm | 14 ABI fault scenarios; subsequent actual Gazebo injection | ABI pass; actual injection awaiting implementation |
| R1: reset/isolation | P6-03/06, SAF-01/08 | Reset clears queued authority; separate model remains independent | ABI replay; two-model/native-reset physics tests | ABI/browser restart pass; native authority reset passes with latched IMU fault; other reset forms/models open |
| R2: wire admission | P4-01/02/03, SAF-03/04/05 | Bounds/authentication before mutation; rejected traffic does not renew a lease; full queue cannot displace stop | Protocol/owner tests, signed-client sessions | Host tests pass; provisioning/real-client sessions awaiting integration |
| R2: reboot replay | P4-04/05, SAF-03/05 | Prior-boot signed traffic cannot restore authority; enrollment failure denies radio | Provisioning ADR, failure cases and hardware traces | Awaiting accepted provisioning workflow |
| R3: exact board | P0-02/04, P3-01/03 | Identified sensor/axes, conflict-free verified pins and stopped first boot | Board record, IMU readback, scoped GPIO/PWM traces | Awaiting actual board, wiring and instruments |
| R4: battery/loss | P8-01/02, SAF-01/03/05 | Invalid/stale battery cannot appear healthy; agreed voltage/source-loss action reaches final gate | Policy ADR, boundary tests, measured ADC/voltage | Awaiting electrical/airframe facts and accepted policy |
| R5: physical stop | P3-05, P8-03, SAF-01/02/06/08 | Measured PWM/gate stop in boot/panic/stall/sensor/radio/diagnostic failures within accepted bound | Props-removed logic-analyzer/scope traces | Awaiting hardware; no compile/simulation substitute |
| R5: timing/resources | P0-05, P3-04/06 | 30-minute 1 kHz workload meets accepted acquisition/stop/timing/memory limits | Raw target distributions and missed-deadline counters | Awaiting workload/profile; existing budgets provisional |
| R5: persistence | P5-03/04, SAF-07 | Interrupted writes retain validated recoverable record; invalid records fault; stopped-only writes | Host CRC/dual-slot tests; actual NVS interruption/rollback | Host pass; physical evidence awaiting board |
| R7: extra transport | P7-02 through P7-06, SAF-03/04/05 | Bounded peer/framing/queue behavior, end-to-end stop, measured coexistence | Serial/proxy tests, radio traces, named profile record | SBUS/peer host tests pass; proxy/coexistence/provisioning open |
| R6: release | P8-04/05/06, SAF-01 through SAF-08 | Exact supported profile, accepted envelope, reproducible source/build/recovery and no unresolved mandatory findings | Operator log, manifests, support matrix, review decision | Awaiting preceding gates and operator evaluation |

See the [R1 experiment report](evidence/simulator-r1.md) for raw traces, hashes,
commands and failed comparison findings. See the [Bangladesh BOM](bangladesh-hardware-bom.md)
for sourcing candidates; no exact physical profile is qualified.

## Measurement contract for the first two physics probes

- Gazebo 11, 65 g legacy model, ODE, 1000 µs physics step; 20 ms intended command
  interval, captured at actual plugin receive/processing simulation timestamps.
- Existing default estimator/gains and motor asymmetry remain unchanged.
- Ground center spawn 0.0138 m. Warm up 2 simulated seconds; select STAB and
  explicitly arm after zero thrust. Inputs/durations are in the JSON fixture.
- Actual pose is observational truth, never an estimator correction.
- Trace includes every successful tick: sequence, simulation time, generation,
  lifecycle/reason/source, true pose/velocity, raw IMU, estimate/rates, controller
  targets/validity and final normalized applied motor values.
- Command events include accepted/rejected admission, simulation receipt time
  and the original unsigned loopback bench MAVLink frame. Never put signing keys
  or production credentials in this development trace.
- Stop latency is measured from plugin admission of disarm to the first zero
  applied-motor tick. Missing sequence numbers invalidate the measured bound.
  Wall-clock timeout is 45 seconds; simulation-time duration is fixture-defined.
- Post-disarm landing may fault; it must remain stopped. Record such faults
  separately from faults during commanded flight.
- The original browser flip has no captured input sequence. Candidate probe
  reproductions cannot be called a replay of that event.

## Trace transport

Opt-in SDF `trace_port` sends to a distinct loopback UDP destination. The plugin
uses fixed stack buffers and `MSG_DONTWAIT`; it never writes trace files. A
collector process validates and writes versioned JSONL records outside flight
control. Disabled tracing has no snapshot or packet cost. Enabled tracing costs
one extra registry read plus one 184-byte packet per 1 ms tick, and up to four
command packets per tick; qualify this as instrumented simulation overhead.
Packets may drop: sequence accounting is mandatory. Initial packets before the
collector binds are outside the measured interval.

`FLXT` v1: little-endian header `<4sIQQIIIIQ` plus 34 little-endian floats.
`FLXC` v1: header `<4sIQII` plus at most 280 frame bytes. Internal quaternions are
wxyz, gyro/rates rad/s, acceleration m/s², position m and velocity m/s.
The virtual IMU sample timestamp is the plugin tick timestamp as in the current
adapter; this does not prove the physical sensor acquisition timing.

## Prerequisites and scope decisions

Hardware-profile and instrument information is missing. No actual pin, battery
threshold, flight-loss action or release profile is invented here. The first
software slices continue independently. Explicit wired/unsigned simulator control
remains development-only. Selecting a narrower first release, discovering new
radio peers or changing controller tuning requires its own recorded decision.
