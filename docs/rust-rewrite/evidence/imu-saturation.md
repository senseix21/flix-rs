# MPU6050 register saturation: detection and operator warning

Date: 2026-10-10. Base: commit `ead357e` on branch `hardening/unsafe-and-properties`.
Requirements: SAF-03 (sensor validity evidence), P3 IMU acquisition, P4 telemetry.
Hardware profile: none. Host tests with a fake I²C bus; Xtensa compile only.

## Problem

At the configured ±2000 deg/s and ±4 g ranges, a reading beyond range returns the register
endpoint (`i16::MIN` or `i16::MAX`). The controller then integrates a clipped rate or acceleration,
and nothing recorded that it happened. Accelerometer saturation is common on landing: the Gazebo
landing run saw 15 limited ticks and a 558 m/s² raw peak. Gyro saturation is rare but can occur
in a tumble.

## Contract

- **Detection:** the driver sets a per-axis bit when a raw register equals `i16::MIN` or
  `i16::MAX`. Gyro x/y/z use bits 0..=2 and accel x/y/z bits 3..=5, matching the Gazebo
  plugin's `sensor_limit_mask`. Axes are in the sensor frame, before bias and rotation.
  The temperature register is ignored.
- **Reporting:** `Counters::imu_saturated_samples` counts saturated samples per axis
  (saturating u64). STATUSTEXT 253, severity WARNING (4), reads `IMU saturated:` followed by
  the axes. At most one notice per second names every axis that saturated since the previous
  notice. A full reply queue defers the notice instead of dropping it. `Owner::reset` discards
  pending notices; counters live for the owner object's lifetime.
- **Control is unchanged:** the controller receives exactly the value it received before this
  change. Whether to fault, de-weight the accelerometer or stop on saturation is a
  flight-safety decision that needs its own experiment.

## Dialect

`253: [83, 51, 54]`, from the legacy-pinned MAVLink 2.0.33 header. Cross-checked against current
`c_library_v2` (SHA-256 `1767e8b5…06e6`, same values). It is outbound only; an inbound 253 is
rejected as unsupported.

## Verification

| Check | Result |
| --- | --- |
| Driver: every axis at both endpoints maps to its bit; one count inside never sets it; temperature ignored | Pass |
| Driver: saturated sample value unchanged (`i16::MAX` gyro reaches the controller as before) | Pass |
| Owner: landing-style burst plus a gyro spike gives two notices 1 s apart; second names `gx az`; counters `[1,0,0,0,0,15]`; no overflow | Pass |
| Owner: continuous saturation gives one notice per second; reset discards pending | Pass |
| C oracle: STATUSTEXT payload for all 63 non-empty masks matches `mavlink_msg_statustext_pack` | Byte-identical |
| `scripts/check.sh` | Exit 0: 66 Rust, 35 Python, 400 parity cases |
| Xtensa archives; `scripts/deny.sh` | Build; all four workspaces pass |
| Fuzz `owner` with `Saturation(u8)` op (including undefined high bits), 300 s | 1,885,483 runs, 0 crashes |

## Cost

The added per-sample work is six integer comparisons and a fold in the 1 kHz acquisition path,
plus one comparison in the owner tick. It has not been measured on target; the R5 timing run
must include it.

## Unresolved

- The Gazebo ABI `Input` (48 bytes, version 1) does not carry the plugin's mask, so simulated
  saturation is visible in traces but not in owner counters or STATUSTEXT. Carrying it requires
  an ABI version change.
- The counters are not exported yet. The frozen 52-value metrics snapshot would need a version 2.
- Physical saturation behaviour, including whether the MPU6050 holds exactly at the endpoint
  under DLPF=1, is unverified on hardware.
- QGroundControl display of the warning has not been checked manually.
