# Qualification and recovery procedure

Status: no flight-qualified profile. Do not enable physical outputs from a host test result.

## Profile record

Record actual board SKU/revision and chip/module marking, IMU identity/register readback, sensor-to-body rotation, bus pulls/voltage and pins, gate circuit and external pull-down, motor/driver order and polarity, airframe mass, props, power/battery circuit, exact firmware/config hashes, SDK/compiler and transport. Pins remain disabled until this record exists. The S3 development board proposal is a bench candidate, not an accepted airframe.

## Props-removed bench gate

1. Build with `scripts/build-esp32.sh` on ESP-IDF v5.4.2 and the pinned Xtensa compiler; save compiler/linker output, map and binary hashes.
2. Initially isolate motor power. Check every output and active-high gate through boot, startup failure, reset and panic using an oscilloscope/logic analyzer. External pull-down must stop motor power while the CPU resets.
3. Configure/read back the MPU6050. Record acquisition start/end/sample times and stationary calibration behavior. Measure stale/missing/disconnected sample handling.
4. With propellers removed and the verified gate circuit, measure arming denials, USB/signed-radio disarm, every flight intent, maintenance expiry, stale cached frames, clock gaps, loop stalls, watchdog/panic/reset and queue/radio overload. Capture actual gate and PWM traces. A reset alone is insufficient evidence.
5. Run 30 minutes at 1 kHz with the declared sensor rate, maximum legal command traffic, full telemetry and adversarial packet load. Record all deadline misses, jitter/latency distributions, stack/heap high-water marks and minimum free memory. Proposed budgets are 500 µs core, 1000 µs sample-to-output, 100 µs jitter; the SDK timer/driver strategy is unaccepted until measured. Output expiry is provisionally 2 ms plus the 250 µs timer service interval; it is not a measured guarantee.
6. Measure voltage and power behavior. The battery adapter and low-voltage flight policy are open; unknown telemetry must not be treated as a valid battery measurement.
7. Validate actual pyflix/QGroundControl signing provisioning, replay across reboot, ACK timing, source loss/takeover and reconnect. Radio is disabled until a trustworthy per-boot time floor is supplied. ESP-NOW is qualified independently.

## Operator-controlled flight gate

Accept the preceding traces, airframe power/gate design, calibration, motor directions, timeout policy, timing thresholds and remaining findings first. The operator then performs restrained and limited flight evaluation. Record the flight envelope, environment, durations, loads, faults, command/output traces and each tuning change. This repository does not authorize or automate flight.

## Recovery and release

Keep the last verified binary, partition layout, both exported configuration records and a manual recovery procedure. With motor power isolated, use ESP-IDF flashing/recovery tools to restore the known image; verify all output pins remain stopped before restoring configuration. Export settings before changes. Reject unknown schema versions; invalid records latch configuration fault. Never erase storage automatically to conceal corruption.

Generate artifact metadata with `python3 scripts/release-manifest.py path/to/artifact --profile NAME`. It labels uncommitted/unqualified artifacts honestly. A release review must attach exact source revision, lockfile/schema/dialect/SDK hashes and hardware evidence, plus a support matrix and recovery result. Physical thresholds, signing provisioning and battery policy remain open decisions until recorded.
