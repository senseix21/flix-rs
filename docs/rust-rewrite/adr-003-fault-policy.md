# ADR 003: Fault policy and recoverability

Status: accepted 2026-10-10; D1–D3 implemented, D4 and D5 open. Supersedes no ADR. Requirements: SAF-01, SAF-03, SAF-05, SAF-07,
P5 settings integrity, P8 operator recovery.

## Context

Fault behaviour is decided function by function, with no single policy.

- `clear_fault()` refuses `Configuration` and generation exhaustion with an inline check. `reset()`
  has no such check, so it returns any fault to Boot. The only caller of `reset()` today is the
  Gazebo `flix_reset`, where a configuration fault cannot arise, so this is a latent defect, not a
  reachable one.
- The same physical failure has two labels. If the MPU6050 driver fails `configure()` at boot,
  firmware raises `SensorExpired` (clearable). If it fails after a maintenance driver rebuild,
  firmware raises `Configuration` (never clearable on board). Either way the operator is told the
  wrong cause.
- `Reason` mixes three kinds of value: faults (`Tilt`, `Clock`, `InvalidNumeric`, …), stop reasons
  that are not faults (`CommandExpired` puts the owner in Failsafe; `MaintenanceExpired`), and
  status (`None`, `Disarmed`, `NotReady`).
- The operator never sees a reason over MAVLink. HEARTBEAT reports only
  `MAV_STATE_CRITICAL` (5) for Fault, otherwise STANDBY (3). Denied arm and `clear-fault` requests
  return a bare ACK or nothing. The reason reaches only the Gazebo ABI and traces.
- `Flight` (portable core) holds a configuration-integrity fault that only the board layer can
  detect or resolve. That fact belongs to the platform, not to flight-time policy.

## Fault inventory (current code)

| Reason (code) | Kind | Raised by | Resulting state | Cleared today by | Proposed class |
| --- | --- | --- | --- | --- | --- |
| `None` (0) | status | arm, maintenance | — | — | status |
| `Disarmed` (1) | status | disarm, first healthy sample | Standby | — | status |
| `NotReady` (2) | status / fault | boot, reset, clear; also armed while uncalibrated (fault) | Boot / Fault | `clear_fault`, `reset` | recoverable when a fault |
| `CommandExpired` (3) | stop | tick: lease expired while armed | Failsafe | `disarm` | stop (not a fault) |
| `SensorExpired` (4) | fault | tick: stale/missing/non-monotonic sample; firmware boot driver failure | Fault | `clear_fault`, `reset` | recoverable |
| `InvalidNumeric` (5) | fault | estimator or controller non-finite | Fault | `clear_fault`, `reset` | recoverable |
| `Clock` (6) | fault | tick: zero, backward or > gap step | Fault | `clear_fault`, `reset` | recoverable |
| `Generation` (7) | fault | generation counter at `u64::MAX` | Fault | nothing in `clear_fault`; `reset` returns early | latched (unreachable in practice) |
| `Tilt` (8) | fault | STAB tilt above limit | Fault | `clear_fault`, `reset` | recoverable |
| `MaintenanceExpired` (9) | stop | diagnostic permit expired | Maintenance | — | stop (not a fault) |
| `Configuration` (10) | fault | firmware: corrupt settings record at boot; driver rebuild failure | Fault | **`reset` only** (inconsistent) | latched |
| `Power` (11) | fault | owner: battery requires stop while armed | Fault | `clear_fault`, `reset` | recoverable; arm stays gated on battery health |

"Recoverable" means `clear_fault` returns to Boot. Arming still requires a fresh healthy sample,
calibration, a fresh low-thrust lease and explicit `arm`, so clearing a fault never re-arms.

## Decisions

**D1. One latching predicate.** Add `Reason::latched()`, true for `Configuration` and
`Generation`. Both `clear_fault()` and `reset()` consult it: from Fault with a latched reason,
neither leaves Fault. A Kani proof is written first, checked to fail on the current `reset()`, and
then must pass: "from Fault with a latched reason, no `Flight` operation leaves Fault".
Consequence: a Gazebo `flix_reset` of a latched owner stays faulted, and the simulator recreates
the owner (`flix_destroy` + `flix_create`) to recover, as a reboot would.

**D2. Name sensor-driver failures by cause.** Add `SensorInit` (code 12), class recoverable, for
`Mpu6050::configure()` failure at boot and after a maintenance rebuild (currently `SensorExpired`
and `Configuration`). `clear_fault` on `SensorInit` requests a driver rebuild. The fault remains
until the rebuilt driver passes `configure()` and readback, so clearing never hides a dead
sensor. Reason codes are ABI (Gazebo `Output.reason`, `FLXT` traces, `gazebo_trace.py` rejects
`reason > 11`), so the range checks are updated in the same change.

**D3. Tell the operator why.** On every entry to Fault or Failsafe, and on every denied `arm` or
`clear-fault`, emit STATUSTEXT (253) naming the reason, e.g. `fault: tilt`,
`arm denied: battery`, `clear denied: configuration (reflash)`. Use the existing deferral so
replies are not lost. Rate-limit repeats of the same text to 1 Hz. HEARTBEAT is unchanged.

**D4. Board-level safe boot for configuration integrity. Design now; build at R3.** With a
corrupt settings record, the board should start in a limited boot mode that permits wired
maintenance, settings export and reflash only, and never builds an armable `Flight`. That makes
"cannot fly with invalid configuration" a property of construction, not of a clearable flag, and
moves the fact to the layer that owns storage. Building it now would add firmware startup code
that cannot be exercised before a board exists. Until then, D1 keeps the existing flag latched.

**D5. No on-board recovery command for corrupt settings (yet).** Reflash with ESP-IDF tools stays
the only recovery, as in [qualification](qualification.md) ("never erase storage automatically").
Revisit with a defined operator workflow; any future path must be wired-only, stopped-only,
explicitly confirmed, write both slots and require a reboot.

## Consequences

- Recoverability becomes one reviewed classification. Adding a reason means adding a row here and a
  `latched()` decision.
- `reset()` becomes stricter. Simulator tests that reset a faulted owner and expect Boot must
  recreate the owner for latched reasons. No current test does.
- STATUSTEXT traffic increases only on transitions and denials; steady state is unchanged.
- `Reason` still mixes status and faults in one enum, because it is ABI. A future ABI version may
  split it; this ADR does not.

## Evidence plan

| Decision | Check |
| --- | --- |
| D1 | Kani: latched faults never leave Fault through `clear_fault`, `reset`, `select`, `maintenance`, `disarm`, `reconfigure` (run first against current code; must fail on `reset`) |
| D2 | Host test with the fake bus: configure fails, then `SensorInit`; clear while failing stays faulted; bus recovers, clear succeeds to Boot and still requires explicit arm |
| D3 | Owner tests: each fault entry and each denial emits exactly one STATUSTEXT with the expected text; repeats rate-limited; no `reply_overflow` at default telemetry |
| ABI | `gazebo_trace.py`, `abi-smoke.py` and `simulator-contracts.py` accept code 12; Gazebo CI scenarios pass |
| D4, D5 | Recorded open items for R3 |

## Alternatives considered

- **Keep `reset()` as a full clean slate.** Simplest, and matches the simulator's current use. But the
  latching rule stays unenforced and any future caller (a watchdog recovery, a console reset) could
  silently clear a configuration fault.
- **Make `reset()` re-validate storage (reboot semantics).** Closest to hardware, but `Flight`
  cannot see storage. The correct home for that is the board-level boot mode in D4.
- **Treat every fault as latched until reboot.** Maximally conservative, but turns transient bench
  events (a single clock gap, a loose sensor cable) into power cycles, and teaches operators to
  power-cycle reflexively.

## Implementation record (2026-10-10)

| Decision | Change | Evidence |
| --- | --- | --- |
| D1 | `Reason::latched()`; `clear_fault()` and `reset()` both consult it | Kani `latched_faults_never_leave_fault` failed on the old `reset()` and passes now; owner test: Configuration survives clear and reset, Tilt/Clock/Power clear |
| D2 | `Reason::SensorInit = 12` for `configure()` failure at boot and after rebuild; settings that cannot build a driver stay `Configuration`. `clear-fault` sets `sensor_retry_requested`; firmware rebuilds and calls `Owner::sensor_retry_result` | Owner tests: success clears to Boot and still needs explicit arm; failure stays faulted; a late success cannot clear a newer Configuration fault; the test fails if clear-fault clears SensorInit directly. Trace decoder accepts 12 and rejects 13 |
| D3 | `Reason::label()`, `ArmBlock` and `Flight::arm_blocker()` (which `arm()` now uses, so the cause cannot drift). Owner queues notices (4 slots, identical text suppressed for 1 s), flushed before periodic telemetry: `fault: <reason>` (CRITICAL), `failsafe: <reason>`, `arm denied: <cause>`, `clear denied: …` (WARNING) | Owner tests for each message, severity, 1 s suppression and no reply overflow; the generic STATUSTEXT encoder matches all 63 C-library fixtures; the owner fuzz target validates every notice against this vocabulary (1.32M runs, 0 failures) |

The firmware boot path keeps its existing fallback to identity rotation for unreadable rotation
settings. Only its fault reason changed. All gates pass: `scripts/check.sh` (75 Rust, 37 Python,
400 parity cases), 6 Kani proofs, both Xtensa archives, and all fuzz targets.

Both D3 test failures during development were test mistakes (a 1 s time jump and a repeated
timestamp, both clock-gap faults). The new notices named them (`fault: clock`).
