# Software acceptance: remaining five items

> **Historical record.** For the current state see [status](status.md).

This work follows `AGENTS.md`. It does not qualify a physical aircraft.

[Evidence index](evidence/software-five/index.json) records raw traces, logs and hashes.

## Contracts before final verification

1. **Landing:** emulate the MPU6050's configured ±4 g and ±2000°/s per-axis measurement range at the Gazebo sensor boundary. Preserve nonfinite rejection and the core's numerical limits. Trace version 3 retains the raw physical input and six saturation bits. A ground-contact scenario must observe range limiting without entering Fault. This is a range model; filtering, noise, ADC quantization and physical mounting remain unmeasured.
2. **Behavior comparison:** preserve the full predeclared C++/Rust thresholds and all violating ticks. Classify dispatch differences only on actually logged MANUAL_CONTROL transitions; require stable intervals to meet the original limits. Full tick parity is a separate gate. Add twelve-second level flight, roll/pitch/yaw pulses and the previously failing contact sequence.
3. **Actual client:** test the pinned QGroundControl v5.0.8 executable against the real signed Rust owner. Its signing setting is a passphrase whose SHA-256 digest is the wire key. `tools/enroll-radio.py --qgc-passphrase-file` creates a fresh private passphrase and per-boot session. The loopback bench can announce telemetry to an explicit loopback endpoint without admitting unsigned commands. Record which actual QGC requests are admitted and which features remain unsupported.
4. **Failures:** exercise signed Stop through both bounded ingress queues, replay rejection after Stop, queue reset, reboot and invalid/clock-discontinuous session rotation. Retain the existing real Python serial → Rust bridge → signed owner, disconnect, wrong-key and provision-validation checks. These checks do not establish physical ESP-NOW callbacks or RF behavior.
5. **Delivery:** package complete SDK flash file sets, configuration, source archive and source hashes. Verify inventory, paths, hashes and nonoverlapping flash spans before installation. Atomically stage a package, preserve the previous verified package and support local rollback. Never include private peer provisioning or fresh session keys. Local rollback changes the software package; board rollback requires physical verification.

## Final verification

Implementation was followed by the final checks, with focused reruns after
compatibility and harness defects were found. No board was flashed.

| Check | Observed result |
| --- | --- |
| Formatting, generated schema/dialect, Clippy | Pass |
| Rust host tests | 49 pass |
| Python tests | 34 pass |
| Pinned legacy scalar oracle | 400 pass; maximum absolute error 4.77e-7, limit 5e-6 |
| Simulator ABI safety scenarios | 14 pass |
| Actual Gazebo scenarios | 20 pass, with complete final traces |
| MPU6050 range/applied-output trace audit | All 20 final traces pass |
| Portable firmware/proxy `no_std` checks | Pass, `thumbv7em-none-eabihf` |
| Exact SDK target builds | Proxy, default flight, ESP-NOW flight pass; ESP-IDF v5.4.2, Xtensa Rust 1.88.0.0 |
| Actual QGroundControl | Signed discovery, capability/protocol negotiation and all 96 parameter values observed; internal readiness check recorded in final client evidence |
| Complete development package | App/bootloader/partition/config/flash files, source snapshot and hashes implemented |
| Installation/rollback | Verified staging, replacement and rollback pass; corrupt/traversal/symlink inputs rejected |
| Full C++/Rust plant parity | Fail under the original limits; details below |

Host commands: `sh scripts/check.sh`, plus the two separate firmware Cargo
manifests checked with `--target thumbv7em-none-eabihf --locked --release`.
Exact SDK commands use `scripts/build-proxy.sh`, the locked `cargo +esp rustc`
flight archive build, and isolated `idf.py -B` default/ESP-NOW build directories.
The Gazebo commands are the 20 `make sim-scenarios` isolated-container cases,
using the cached image after a registry metadata stall. No test connects to the
browser's live container. See the evidence index for commands, logs and hashes.

## Landing and trace integrity

The historical raw ground-contact sample exceeded the core's 160 m/s² numerical
limit. The real configured MPU6050 has a per-axis ±4 g measurement range.
The new range model caps finite measurements at the signed-register endpoints,
while preserving nonfinite faults. This fixes the simulator/producer mismatch;
it does not change the core's fault limit.

The landing-contact run observed 15 limited ticks, a maximum raw acceleration
norm of 558.37 m/s², explicit rearming, and no contact or post-Stop fault.
The level and three-axis scenarios also retain raw impact samples, including
larger physical impulses. Inputs remain within the declared per-axis range.

The first two-model run lost 788/834 tick packets under concurrent host load.
Its Stop checks passed, but its trace-completeness gate failed and is retained.
The final run uses a larger buffered trace writer and 500 wall updates/second
with the same 1 ms physics step: simulated control timing remains 1000 Hz.
It records zero drops and zero simulation-tick Stop latency for both models.
This is a simulator scheduling workload; it does not establish hardware timing.

## C++/Rust differences and failed plant parity

Both new Rust nominal flight runs pass their finite envelopes. Both pinned C++
runs record a nonzero motor frame on the observation marked stopped: the legacy
loop computes control before processing Stop. Rust deliberately stops before
applying output. This is a retained safety correction.

| Paired run | Max attitude difference | Max motor difference | Max position difference |
| --- | ---: | ---: | ---: |
| Level | 0.2723° | 0.4733 | 0.0913 m |
| Pitch pulse | 1.0507° | 0.5101 | 0.8990 m |

Original limits remain 5°, 0.1 and 0.5 m. Neither full-tick comparison passes.
The new strict transition classification also fails; it is not widened to turn
these runs into passes. Level violations occur at relative ticks 6, 504–506;
its recorded thrust transitions arrive four simulation milliseconds apart.
The pitch pulse starts one millisecond apart and has a one-tick duration
mismatch; its position separation exceeds 0.5 m after relative tick 2387.
These measured delivery differences constrain the experiment's interpretation.

Legacy acceleration retains its 0.1 filter; Rust uses the configured MPU6050
range model. Rust clamps stored PID integral state and explicitly gates Stop;
legacy does not share those semantics. Truth-yaw injection is disabled only in
the disposable legacy comparison copy. These differences are documented, not
claimed to fully explain the later position separation. Identical input replay
and estimator-state comparison are the next steps for full plant parity.
No controller gains or numerical tolerances were changed to obtain a pass.

## QGroundControl interoperability

The pinned executable is QGroundControl v5.0.8, SHA-256
`1700a82754c47e77cdd1a562267524c85ac021572319eddf7d2ed8287483fce0`.
The actual client discovered system/component 1/1 and sent valid signed frames
from 255/190. The owner answers AUTOPILOT_VERSION and PROTOCOL_VERSION, reporting
only implemented capabilities and MAVLink 2 only. The client requests the full
parameter list and receives indices 0–95.

Its signing setting hashes the passphrase with SHA-256. The pinned version
initializes its epoch from local 2015-01-01 midnight; `TZ=UTC` for that process
aligns it with the enrollment tool. Signed discovery failed in the earlier
UTC+6 experiment and passed with this correction. The initial Cocoa-only
platform failure and early/startup-limited attempts are retained as attempts,
not successful client gates.

Primary implementation references: [signing epoch/key conversion](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/src/MAVLink/MAVLinkSigning.cc),
[link signing policy](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/src/Comms/LinkInterface.cc),
and [protocol/capability negotiation](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/src/Vehicle/InitialConnectStateMachine.cc).

Unsupported mission/navigation requests remain rejected and are not advertised.
Manual QGC flight-control UI acceptance, joystick mapping, physical radio/USB
behavior and successful arming on an inspected board remain open. The normal
Python/browser control path continues to have its own tested contract.

## Delivery limits

The package is development-only. Its hashes provide integrity relative to a
trusted local manifest; they are not a signed public release. SDK config and
motor/pin choices remain compile profiles awaiting physical inspection.
Private peer NVS and per-boot session material are not packaged.
Local rollback swaps verified packages and does not flash or verify board recovery.
A crash between rename steps can leave `.swap`; preserve and verify that recovery
copy rather than overwriting it. Byte-identical target rebuilds, hardware NVS
power-interruption, physical output-stop budgets and flight qualification remain
separate gates. Remote CI and publication have not been performed.


Markdownlint and mdBook are unavailable on this host; neither documentation build was run.


## Final packaged-source and recovery results

The final source archive was verified, extracted into a new directory and built
with an empty target directory. `sh scripts/check.sh` passed there: 49 Rust tests,
34 Python tests, 400 scalar comparisons, ABI safety checks, formatting and Clippy.
The legacy scalar oracle used the explicitly pinned sibling via `FLIX_LEGACY`.

The final local package is `.local/software-five/package-final`; the activated
verified copy is `.local/software-five/install-final`. Its proxy/default-flight/
ESP-NOW app images are 696224 / 441568 / 940832 bytes. Each profile includes all
factory app, bootloader, partition, flash metadata and configuration files.
The SDK's 1 MiB application partition checks passed. Private NVS images are
excluded; the packager permits only the selected factory flash layout.

Installation preserved the previous package, rollback restored it, and a second
verified swap reactivated the newest package while retaining the baseline.
No board was flashed. [Final verification](evidence/software-five/final-verification.json)
and [clean build log](evidence/software-five/clean-host-final.log) record hashes,
profile sizes and results. Post-build evidence is adjacent to the frozen package;
its source inventory remains the exact archive built in the clean directory.

The live browser simulator was restarted only after it was stopped with zero
motors. It loaded the current plugin, returned to the origin, and reports fresh
telemetry, disarmed state, no fault and four zero outputs. The viewer remains at
`http://127.0.0.1:8765/?v=5`.
