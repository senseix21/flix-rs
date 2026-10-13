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
| Parameters | All 96 legacy names inventoried, plus 21 `POS_*` (swarm S2; settings record v2, v1 migrates); strict finite/integer/cross-field validation | Writes only in stopped maintenance; a refused write sends STATUSTEXT `param denied: <why>` (legacy was silent) |
| Storage | Version/CRC/two slots/readback; maintenance-only writes | Actual NVS/power interruption unmeasured |
| Logs | LE versioned records, offsets/counts/completeness checked | New schema; use new downloader/converter |
| Python | Waiter-before-send, serialized writes, bounded callback worker, close/context manager | Same-key concurrent requests denied explicitly |
| Upstream pyflix (`1f5b268`, pymavlink 2.4.47) | Telemetry, parameters, console (`> command` echo as legacy; SERIAL_CONTROL device 0 or 10; NUL-terminated text), DO_SET_MODE with base mode 0 or 1, arm/disarm, MANUAL_CONTROL, SET_ATTITUDE_TARGET | Works unmodified: the wired policy accepts sender 255/0 (pymavlink default) as well as 255/190; the signed radio policy stays 255/190 only. RAW mode unsupported; SET_ACTUATOR_CONTROL_TARGET ignored (diagnostics are maintenance console only); no signing support, so wired bench policy only. Evidence: `evidence/r2-clients/` |
| Wi-Fi | Bounded UDP, endpoint learned after validation, signed command policy | Denied before trusted key/time-floor provisioning; QGC hardware interoperability open |
| RC | SBUS flags, channel/calibration bounds, explicit stopped takeover | First eight calibrated channels; receiver wiring/coexistence open |
| ESP-NOW | Four provisioned encrypted peers, bounded callback queue, overload stop | Optional SDK adapter unqualified; ≤250-byte compatibility profile |
| IMU telemetry | SCALED_IMU kept; HIGHRES_IMU (105) added at the same MAV_RATE_IMU | Upstream pyflix still reads SCALED_IMU; QGC inspector display unverified |
| IMU saturation | Per-axis register-endpoint counters; STATUSTEXT warning at most 1 Hz | Detection only; no control response. Gazebo reaches the owner through ABI v2 (`flix_tick2`) |
| Telemetry rates | Due streams deferred, not dropped, when reply slots are full | Legacy delivered RC_CHANNELS_RAW at ~8 Hz of 10 Hz configured; target UART capacity unmeasured |
| Gazebo | Per-instance handles; same output gate; no truth-yaw correction | Gazebo 11 plugin compiled and loaded; parameter/disarm round trip passes; flight scenarios open |

This is a software development slice across Phases 0–8. Hardware-dependent tasks and unfinished target interoperability are named in the implementation status; the phase gates are not all closed. The original 9–12 week qualification estimate is not replaced by host code generation.
