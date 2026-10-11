# Remaining-phase execution report

> **Historical record.** For the current state see [status](status.md).

Subsequent delivery: [reset, instance isolation and timing capture](reset-metrics-integration.md).
The counts and sizes below describe the preceding integration pass. The newer
report records current verification and supersedes the wired-metrics software gap.

Date: 2026-10-05. Repository: `flix-rust`. Local uncommitted development;
no physical upload, flight qualification, publication or remote CI execution.

This delivery implements bounded software slices of R0–R7 from the
[execution plan](execution-plan.md). It does not close all original phase gates.
The [development runbook](development-runbook.md) gives reproducible commands.

## What changed

| Work | Delivered implementation | Gate still open |
| --- | --- | --- |
| R0 | Numeric 30-minute/1 kHz workload, schema-default telemetry rates, pin/electrical record, source/configuration manifests | Actual exported settings and instrumented workload |
| R1 | STAB, ACRO, AUTO, pitch pulse, loss/reconnect and native-reset/recovery scenarios; all applied outputs traced | Full plant parity, saturation/transition/injection scenarios, time-only/model-only resets, two actual models and sustained-flight/flip acceptance |
| R2 | Maintenance-only trusted UART SETUP_SIGNING, rotation/revoke, fresh timestamp anchor, signed radio replies, endpoint invalidation, authenticated Python reader and private enrollment tool | Physical UART/radio workload and actual QGroundControl compatibility; persistent unattended enrollment deliberately unsupported |
| R3 | Exact-board template and validator for missing facts, pins, electrical margins, IMU axes and partition capacity | Delivered board/schematic/readback and stopped first boot |
| R4 | Opt-in timestamped battery guard with arm inhibition, debounce/hysteresis, stale/invalid handling, latched power fault and measured-voltage telemetry | ADC acquisition/scaling, accepted thresholds and airframe loss policy, actual voltage traces |
| R5 | Fixed histograms for acquisition, sample age, owner tick duration, total acquisition/owner duration and schedule jitter; stopped-only 52-value FFI snapshot | Wired capture integration, physical gate/PWM commits, stack/heap, stop bound and 30-minute target measurements |
| R7 | POSIX serial client with bounded partial writes/disconnect handling; bounded MAVLink proxy framer that rejects unsigned admission and enrollment forwarding | Actual USB-to-ESP-NOW proxy firmware integration, peer persistence and RC/radio coexistence |
| R6 | Development artifact bundle, source/file hashes, build/enrollment/recovery runbook and explicit qualification status | Complete flash/recovery package from a committed source, fresh physical installation/rollback, operator envelope and release approval |

## Ownership and failure contracts

Enrollment is accepted only from the trusted wired path in stopped exclusive
maintenance without a diagnostic permit. Rotation/revocation clears queued and
cached authority. Radio is unprovisioned at boot. The trusted enrollment tool
generates a new random key each session; the board cannot prove freshness if a
different trusted caller reuses an old key. See [ADR 002](adr-002-wired-session.md).
There is no forensic secret-erasure guarantee. Secret payloads are excluded from
general owner queues and telemetry evidence.

Battery policy is disabled unless explicitly installed. When installed, healthy
fresh voltage is required to arm. Missing/invalid/stale voltage stops an armed
bench profile; critical voltage stops after the configured debounce and latches
`Reason::Power`. Healthy recovery does not arm or clear the fault. No landing
controller or calibrated hardware voltage source is claimed.

Timing recording uses fixed storage and saturating arithmetic. The `core`
histogram currently measures the whole owner tick including dispatch and
telemetry. The owner snapshot API is a stopped instrumentation boundary, not
physical stop evidence. The host benchmark reports 496-byte control core and
67,320-byte owner; these are object sizes, not target stack high-water marks.

## Verification

Evidence and command logs are saved under
[`evidence/remaining-execution/`](evidence/remaining-execution/summary.json).

* Consolidated host checks: generation, formatting, Clippy, 41 Rust tests,
  24 Python tests, release build, 400 actual C++ scalar oracle cases, 14 ABI
  failure scenarios, simulator contract checks and host benchmark.
* Portable firmware dependency check: `thumbv7em-none-eabihf` with `no_std`.
* Actual Xtensa archive and ESP-IDF v5.4.2 links: default unwired/PWM-disabled
  image and separate compile-only PWM/RC/Wi-Fi profile. No flashing performed.
* Six finite actual Gazebo scenarios pass: level STAB thrust, pitch pulse,
  native reset with explicit recovery, level ACRO, level AUTO, and command lease
  loss/reconnect. Each final capture has zero missing tick sequences and zero
  invalid trace packets; stop is applied in the receipt tick. Failed earlier
  captures are preserved and never count as passed measurements.

The default image is 439,088 bytes (`0x6b330`), with 58% of its 1 MiB app
partition free. The compile-only enabled image is 999,344 bytes (`0xf3fb0`),
with 5% free. That profile uses arbitrary test pins and dummy credentials;
neither establishes a usable hardware configuration. Flash/partition review
remains mandatory for the delivered board.

## Findings retained

The previous paired Gazebo motor comparison fails around command dispatch despite
the nominal scalar oracle passing. Intentional stop/authority ordering remains;
gains and plant parameters were not tuned to hide the difference.

Native world reset restores upright origin and clears source authority, but
virtual IMU transients can latch a fault. The scenario now records that fault
and tests explicit `clear-fault` recovery with no implicit arming. Two failed
trace captures are preserved: a concurrent-build capture and an isolated capture.
Investigation found the collector blocked while waiting for the reset helper;
it now launches the helper separately while continuing to drain trace packets.
The trace completeness threshold was not relaxed.

The original browser flip has no recorded input sequence. These short scenario
passes do not establish its cause or sustained-flight acceptance. Physical
motors, RF coexistence, NVS interruption, panic/stall output timing, battery sag,
restrained flight and rollback remain unverified.

## Language boundary and next work

The flight application and portable control/services are Rust. C implements the
ESP-IDF adapter; C++ implements the Gazebo adapter. They remain explicit audited
boundaries under ADR 001, not a completed conversion to an exclusively Rust repo.

The next software gates are reset variants/two actual models, broader adverse
physics probes, actual proxy firmware integration and a wired metrics collector.
Gazebo's model-reset hook is documented in its
[Model implementation](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/Model.cc);
testing that path remains separate from the world-reset evidence captured here.
Hardware gates require the exact delivered board, motor/power circuit, pin
mapping and instruments. Actual QGroundControl requires an installed client and
recorded sender/signing evidence. No whole-plan completion claim is made.
