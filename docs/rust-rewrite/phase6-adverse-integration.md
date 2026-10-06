# Phase 6 adverse simulator integration

Date: 2026-10-06. Source: local uncommitted development in `flix-rust`.
This extends [reset/metrics integration](reset-metrics-integration.md).

## Scope and acceptance contracts

The actual Gazebo 11 plugin now exercises four additional finite profiles through
UDP commands and the Rust owner. Fixtures and thresholds live in
`fixtures/simulator/acceptance.json`; `make sim-scenarios` runs these together
with the previous thirteen profiles in isolated containers. The live browser
container is not restarted or updated by this command.

| Profile | Declared behavior |
| --- | --- |
| Saturation/recovery | STAB .55 thrust for 500 ms, full thrust/full pitch for 20 ms, .55 thrust for 500 ms; observe an output at least .999 and later four positive outputs below .95; outputs always finite within [0,1] |
| Mode change | STAB flight for 1 s, explicit Stop, select ACRO and submit zero thrust, remain stopped for 10 ms, explicitly rearm; no automatic rearm during mode selection |
| Timing gap | Omit owner ticks during [3,000,000, 3,011,000) us of actual simulation time; apply zero force throughout; require Clock fault with zero motors on the first resumed owner tick within 1 ms of the declared boundary |
| Diagnostic Stop | Enter stopped Maintenance, test only RL at .10 output for at most 100 ms, observe the requested output, then explicit Stop; remaining motors zero and no later resumed output |

Each ordinary profile retains the 45-degree armed tilt limit, .015 m minimum
lift where applicable, complete 1 kHz trace requirement, and a Stop boundary of
at most 1 ms from admitted command receipt. The diagnostic does not require lift.
Saturation recovery is an observed output return, not a settled attitude or
sustained-flight qualification.

The timing-gap hook is opt-in SDF configuration with tracing required. It does
not invent a separate controller clock: Gazebo advances normally while owner
invocations are skipped. During the gap, trace lifecycle/estimator fields are
the last owner snapshot; input fields describe the constructed sample that was
not consumed, and motor fields describe the zero applied force. Normal/default
SDF has neither this hook nor sensor injection enabled. Reset clears the cached
owner output. This exercises missed owner execution in simulation, not physical
watchdog or external output-gate timing.

## Failures retained and changes

The first mode-change profile stopped only 500 ms after arming and held power
off for 100 ms. An InvalidNumeric fault appeared during ground contact before
rearming. Its raw trace and failed result are retained under `failures/` in the
new evidence directory. The finite transition profile was revised to stop after
1 s and hold for 10 ms, then tested again. Controller gains, estimator tolerances,
and fault policy were not changed. Reliable landing/contact behavior remains an
open physics investigation; the revised profile does not close that gate.

The host allocation test also failed once with six counted allocations. Its
process-wide counter included every test-harness thread. Counting now uses
constant-initialized thread-local cells, scoped to the thread executing the
9,999 flight ticks. The focused test passes; this checks that flight execution
allocates nothing while excluding unrelated harness activity. No production
Rust flight code changed in this pass.

## Verification and evidence

Final regression: all seventeen Gazebo scenarios passed with no missing trace
sequences or invalid trace packets. Each explicit Stop boundary checked in the
new profiles measured 0 simulated microseconds from admitted receipt. The
missed-owner profile recorded eleven zero-force ticks, then Clock fault at
3,011,000 us. This is a simulation-time measurement.

Final results are recorded in [the evidence summary](evidence/phase6-adverse/summary.json),
with compressed raw traces, scenario results, server logs, host/no_std logs,
source fingerprints, configuration hashes, commands, and SHA-256 checksums.
The fixture records seed 24301, but the Gazebo runner does not explicitly set
a physics seed. Do not interpret the results as seeded physics reproducibility.

The host suite reports 42 Rust tests, 28 Python tests, 400 pinned C++ scalar
comparisons and 14 ABI safety scenarios. The separate firmware `no_std` check
also passes. Markdownlint and mdBook are not installed in this environment.

Target firmware builds from the previous pass remain historical evidence.
They were not rerun here: this pass changes simulator/test code, fixtures, and
documentation, not target firmware. No upload or physical test was performed.

## Gates still open

Actual C++/Rust physics parity, landing/contact behavior, longer trajectories,
physical stop and timing measurements, radio coexistence, actual ESP-NOW USB
proxy integration, QGroundControl validation, confirmed wiring and battery
qualification remain open. These bounded simulator profiles do not establish
hardware flight qualification or complete every rewrite phase.
