# Compatibility and behavior changes

| Interface | Implemented behavior | Qualification limit |
| --- | --- | --- |
| STAB / ACRO | Legacy gains, coordinate signs, mixing and desaturation; final bounded outputs | Nominal host comparison; no airframe tuning evidence |
| AUTO | Explicit attitude/rates variants, FRD→FLU conversion, supported masks only | Explicit arm after a fresh low-thrust command; no implicit arm |
| Arming | Health, calibration, source, freshness and low thrust; ACK | No stick arming; boot and reset remain stopped |
| Disarm | Dominates invalid intent and saturated queues; ACK | Physical gate traces still required |
| Source loss | Immediate stop, explicit rearm required | Legacy unqualified descent removed from bench profile |
| Raw/direct motors | Denied from flight protocol | Typed diagnostics only in exclusive maintenance, ≤0.15, ≤100 ms |
| Clock | Integer microseconds and explicit discontinuity fault | 32-bit extension requires polling within half-wrap |
| Parameters | All 96 names inventoried; strict finite/integer/cross-field validation | Schema marks unsupported/immutable fields; no silent acceptance |
| Storage | Version/CRC/two slots/readback; maintenance-only writes | Actual NVS/power interruption unmeasured |
| Logs | LE versioned records, offsets/counts/completeness checked | New schema; use new downloader/converter |
| Python | Waiter-before-send, serialized writes, bounded callback worker, close/context manager | Same-key concurrent requests denied explicitly |
| Wi-Fi | Bounded UDP, endpoint learned after validation, signed command policy | Denied before trusted key/time-floor provisioning; QGC hardware interoperability open |
| RC | SBUS flags, channel/calibration bounds, explicit stopped takeover | First eight calibrated channels; receiver wiring/coexistence open |
| ESP-NOW | Four provisioned encrypted peers, bounded callback queue, overload stop | Optional SDK adapter unqualified; ≤250-byte compatibility profile |
| IMU telemetry | SCALED_IMU kept; HIGHRES_IMU (105) added at the same MAV_RATE_IMU | Upstream pyflix still reads SCALED_IMU; QGC inspector display unverified |
| Telemetry rates | Due streams deferred, not dropped, when reply slots are full | Legacy delivered RC_CHANNELS_RAW at ~8 Hz of 10 Hz configured; target UART capacity unmeasured |
| Gazebo | Per-instance handles; same output gate; no truth-yaw correction | Gazebo 11 plugin compiled and loaded; parameter/disarm round trip passes; flight scenarios open |

This is a software development slice across Phases 0–8. Hardware-dependent tasks and unfinished target interoperability are named in the implementation status; the phase gates are not all closed. The original 9–12 week qualification estimate is not replaced by host code generation.
