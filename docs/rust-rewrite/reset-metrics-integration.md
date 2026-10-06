# Reset, instance isolation and timing capture integration

Dates: 2026-10-05–06. Local, uncommitted development in `flix-rust`.
This follows the [remaining execution report](execution-report.md) and closes
additional software slices of R1 and R5. Qualification remains profile-specific
and requires physical evidence.

## Changes

* The Gazebo adapter now overrides model reset and subscribes to time reset.
  Both revoke owner authority and learned endpoints. Full reset may call both;
  successive generations never restore an output frame.
* Reset closes/rebinds the command socket to discard datagrams still in the
  kernel receive queue. A paused-world regression confirms pre-reset mode/arm
  packets cannot restore source selection.
* The adapter drains bounded replies while no endpoint exists. Old telemetry
  cannot occupy every slot and displace the first ACK on a late connection.
  An actual Python client attaches after warmup and exercises commands and
  the maintenance metrics snapshot.
* Reset scenarios distinguish position-reset and clock-reset contracts. The
  external helper supports all three variants and logs its result while the
  collector continues receiving traces.
* A two-model world exercises separate ports, arm/control and stop authority.
  Stopping A must preserve B's generation and flight state. Both traces need
  complete sequences and stop latency no greater than one 1 ms tick.
* Explicit stale and invalid IMU scenarios inject one input at the fixture's
  timestamp. Normal SDF profiles have no injection. The same owner and final
  output gate process the failure; healthy next-tick input cannot rearm.
* Trace v2 records the input sample timestamp and accepts Power reason 11.
  Historical v1 traces remain readable. Normal sample time is still assigned
  from the simulator tick, not measured physical IMU acquisition.
* The owner freezes timing data only in exclusive stopped maintenance and
  sends at most one bounded SERIAL_CONTROL chunk per tick. Reply capacity
  limits progress; Stop/reset/enrollment/exit cancels the remaining transfer.
* Python correlates a request nonce and eight chunks, tolerates reordering and
  identical duplicates, and rejects missing/conflicting/malformed snapshots.
  `tools/capture-metrics.py` provides the direct UART collection command.

The architecture, wire format and use are documented in the
[adapter README](../../adapters/gazebo/README.md),
[protocol contract](protocol.md) and [development runbook](development-runbook.md).

## Verification and findings

The consolidated host suite passes 42 Rust tests and 28 Python tests, 400 actual
C++ nominal scalar cases and 14 ABI failure scenarios. Formatting, Clippy,
release builds and the portable `no_std` firmware check pass. Metrics tests cover
exclusive maintenance/diagnostic denial, frozen values, Stop cancellation,
wrong request nonce, reordered/missing/conflicting chunks and an actual
Python-to-Rust owner exchange. This is not a physical UART test.

The actual Xtensa archive and both ESP-IDF v5.4.2 profiles link. The default image
is `0x6b890` bytes with 58% of the 1 MiB application partition free; the compile-only
PWM/RC/Wi-Fi image is `0xf4540` bytes with 5% free. Pins remain arbitrary test
values or unwired defaults. Host object sizes are 496 bytes for the control core
and 67,752 bytes for the owner; target stack/heap measurements remain open.

Final physics results, failed reproductions, command logs, manifests and hashed
raw traces are recorded in
[`evidence/reset-metrics/summary.json`](evidence/reset-metrics/summary.json).
All thirteen delivery scenarios pass with zero missing tick sequences: STAB,
pitch pulse, ACRO, AUTO, command loss/reconnect, full/time/model reset,
two-model isolation, invalid/stale sensors, queued-reset and the late Python
client. Both models' measured stop latencies are zero simulated microseconds
from admitted Stop to applied zero output. Reset recovery remains explicit.
These are finite software measurements, not sustained-flight qualification.
The model-only baseline timed out without observing a Rust reset while retaining
complete trace sequences. The explicit reset hooks correct that missing path.
An earlier time-only capture timed out during concurrent compilation and is
retained as a failed result; it cannot establish time-reset behavior.

Two additional failing reproductions were captured before their corrections.
With datagrams queued while the world was confirmed paused, model reset restored
source selection from the kernel receive queue. Closing/rebinding discards that
queue as well as the owner's pending authority. Separately, an actual late
Python client timed out waiting for its first disarm ACK; the owner reply slots
were occupied by telemetry produced before endpoint learning. Draining unsent
replies fixes that bounded queue lifecycle. Neither correction changes gains,
motor ordering or the legacy physics plant.
The exact source snapshots for failing pre-correction runs were not retained;
their commands and raw observations are saved, and this provenance limitation
is explicit in the summary. The final source/configuration hashes are retained.

## Open gates

Full paired plant parity and sustained-flight/flip acceptance are still open.
Mode transitions, saturation/recovery, timing-gap and diagnostic-stop physics
coverage are separate from the nominal-mode and ABI results. Do not tune the
controller to hide the previously documented dispatch difference.

The wired metrics collector does not perform the 30-minute workload, physical
PWM/gate stop, memory high-water or interruption tests. Those need the actual
electrical profile and instruments. Proxy firmware integration, peer lifecycle,
radio coexistence, QGroundControl and physical installation/rollback remain
separate gates. Nothing in this delivery qualifies a flight profile.
