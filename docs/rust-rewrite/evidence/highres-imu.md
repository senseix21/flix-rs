# HIGHRES_IMU telemetry and reply-queue deferral

Date: 2026-10-10. Base: commit `e4490b9` on branch `hardening/unsafe-and-properties`.
Requirements: P4 telemetry, R2-04 client compatibility, R0-03 traffic workload.
Hardware profile: none. Host, simulator-free and compile-only evidence.

## Problem

SCALED_IMU (26) carries gyro as i16 mrad/s, so it saturates at 32.767 rad/s (about 1,877 deg/s).
The MPU6050 is configured for ±2000 deg/s (34.9 rad/s). The controller uses the unclipped f32
sample; only telemetry was affected. The legacy C++ firmware passes the same overflowing float
into an int16 field, which is undefined behaviour in C.

## Decisions (user, 2026-10-10)

- D1: send **both** messages. Upstream pyflix (`tools/pyflix/flix.py:194` at the pinned legacy
  revision) fills `flix.acc` and `flix.gyro` only from SCALED_IMU, so replacing it would break that
  client. Both share `MAV_RATE_IMU`, so the 96-entry parameter schema is unchanged.
- D2: intentional changes are recorded in [compatibility](../compatibility.md).

## Contract

For every IMU sample the owner emits:

- exact f32 acceleration and gyro, FLU converted to FRD as x, -y, -z;
- `time_usec` equal to the sample tick in microseconds;
- `fields_updated` bits 0..=5 set only for finite axes, with non-finite axes sent as 0;
- magnetometer, pressure, temperature and `id` zero.

## Dialect

`protocol/common-subset.json` adds `105: [93, 62, 63]` (CRC extra, minimum and maximum length),
sourced from the legacy-pinned MAVLink 2.0.33 header and cross-checked against current
`c_library_v2` (SHA-256 `cdc669ed…d031`, same values). An inbound 105 frame still yields
`Error::Unsupported`; the message is outbound only.

## Reply-queue finding (existing behaviour)

Profiling at schema defaults (10 s, four replies drained per 1 ms tick):

| Build | Replies dropped in 10 s | Note |
| --- | ---: | --- |
| Before this change | 19 | Every 500 ms, heartbeat, battery, attitude and motors fill all four slots; RC_CHANNELS_RAW is dropped (79 of 100 delivered) |
| HIGHRES_IMU added, drop-on-full | 20 | One extra collision |
| HIGHRES_IMU added, deferral | 0 | Every stream within ±2 of its configured count |

The fix: a due stream that finds too few free reply slots stays due and is sent on a later tick.
Command ACKs are queued earlier in each wake (USB ingress runs before `tick`), so they were not
the dropped messages here; `reply_overflow` now counts only real loss.
`every_stream_is_delivered_at_its_configured_rate` fails on the previous code (RC 79/100) and
passes with the fix.

## Verification

| Check | Result |
| --- | --- |
| C oracle (`scripts/telemetry-oracle.py --check`), 6 vectors incl. ±0, subnormal, `f32::MAX`, NaN/±inf, `u64::MAX` time | Byte-identical |
| `telemetry_oracle` Rust test against the fixture | Pass |
| `highres_properties` proptest (256 cases, any f32 incl. non-finite) | Pass |
| `imu_telemetry` owner tests (rates, no overflow, 1,990 deg/s exact vs SCALED_IMU at i16::MAX) | Pass |
| pyflix `HighresImuTests` (decode, >32.767 rad/s, MAVLink 2 truncated payload) | Pass |
| `scripts/check.sh` | Exit 0: 59 Rust, 35 Python, 400 parity cases |
| Xtensa archives (flight, proxy) | Build |
| Fuzz: owner and wire_valid 300 s each; other four 60 s each | 0 crashes (owner 1,463,287 runs; wire_valid 36,176,858) |

The oracle exposed one subtle fact: MAVLink 2 trims trailing zero payload bytes and writes the CRC
right after `msg.len`. Bytes past `msg.len` in the C buffer are CRC, not payload. The Rust
encoder always sends the full 63 bytes, which is valid; pyflix pads truncated payloads with zeros.

## Measured cost

Unsigned telemetry at schema defaults on the host: **3,492 B/s**, about 30% of 115,200 baud
(previous estimate 3.0 KB/s). The workload fixture now declares 74 messages per second.

## Unresolved

- Target UART and ESP-NOW throughput with signing (+13 bytes per frame) are unmeasured.
- QGroundControl v5.0.8 MAVLink Inspector display of message 105 has not been checked manually.
- Gazebo trajectories exceeding 1,877 deg/s were not run for this change.
- Follow-up: MPU6050 register saturation at ±1,998 deg/s still clips the control input itself.
  Detecting and reporting it needs a separate change; acting on it is a flight-safety decision.
