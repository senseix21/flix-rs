# R2-04: real client compatibility (upstream pyflix)

Date: 2026-10-13. Base: `main` `fdcb651` plus the changes on branch `r2/client-compat`.
Requirements: R2-04 ([remaining plan](../../remaining-plan.md)), base spec section 11 (clear
denials, no concealed rejection). Hardware profile: none (host bench, loopback UDP).

## In plain terms

The original Flix Python client (pyflix) can now control the Rust firmware on the wired
development link: connect, read and change settings, use the console, change mode, arm, fly by
stick commands and disarm. It works unmodified. QGroundControl is not covered here yet.

## Pinned client

| Item | Version |
| --- | --- |
| pyflix | upstream `okalachev/flix` `1f5b268271536e45a9377d1c5317d3eb6f3c3bba`, `tools/pyflix` (package 1.0.0) |
| pymavlink | 2.4.47 (with numpy 1.26.4), Python 3.9.6 |
| Server | `target/release/bench` (Rust owner, BenchUsb policy: sender 255/190, unsigned, loopback) |

Command: `.local/pyflix-venv/bin/python tools/pyflix-upstream-compat.py --legacy .local/legacy-git --output DIR [--sender-component 190]`.

## Findings and changes

Before the changes, with pyflix's sender set to 190, pyflix could connect and read settings but
mode change, console and arming failed. Compared with the legacy firmware (`flix/mavlink.ino`,
`flix/console.ino`) each failure was a Rust-side strictness with no safety purpose:

| Client behaviour | Legacy firmware | Rust before | Change |
| --- | --- | --- | --- |
| `set_mode` sends DO_SET_MODE with param1 (base mode) 0 | ignored param1 | required 1, replied UNSUPPORTED | accept 0 or 1; mode stays param2; other params must be 0 |
| `cli` sends SERIAL_CONTROL device 0, 70 bytes zero-padded, `count` 70 | accepted any device; text ended at the first NUL | required device 10; the padding made every command unknown | accept device 0 or 10; text ends at the first NUL |
| `cli` waits for `> command` echo | echoed `> command` then output | replied only to `status`, without echo | every console command replies `> command` newline result (`ok`, `denied: ...`, state) |
| `set_param` outside maintenance waits for PARAM_VALUE | silently ignored a failed write | silently ignored | STATUSTEXT `param denied: maintenance only` (or `unknown name`, `out of range`, `conflicts`, `not supported`); still no PARAM_VALUE, so the client reports failure |

The repo's own client (`tools/pyflix/client.py`) matches echoed replies to their command, so
`status()` cannot take another command's echo.

## Results after the changes (`pyflix-component-190.json`)

21 steps. 19 succeed; the 2 that fail are deliberate refusals with a clear reason:

| Step | Result |
| --- | --- |
| connect, heartbeat mode/armed | ok (`STAB`, disarmed) |
| `get_param MAV_SYS_ID` | 1.0 |
| `set_param` while stopped (not maintenance) | **refused**: pyflix logs `Flix #1: param denied: maintenance only` and raises after its retries |
| `cli status` | `stopped` |
| `set_mode STAB`, `set_controls` throttle 0, `set_armed True` | ok; heartbeat reports armed |
| `set_controls` 0.4, `set_armed False` | ok; heartbeat reports disarmed |
| `cli maintenance`, `set_param CTL_ATT_MAX 0.5`, `get_param` | ok, reads back 0.5 |
| `cli exit` | ok |
| `set_mode RAW` | **refused**: `MAV_RESULT_UNSUPPORTED` (diagnostics are maintenance-console only) |
| `set_mode AUTO`, `set_attitude` thrust 0 | ok |
| `set_motors` | sent; SET_ACTUATOR_CONTROL_TARGET is not decoded (counted as rejected); pyflix does not wait for a reply |
| telemetry (attitude, gyro, motors) | received |

## Sender identity (decided 2026-10-13)

Before: with pyflix's default sender (255, component 0 from pymavlink) the bench refused every
command. The user chose to accept component 0 on the unsigned wired policy only: on that path
the sender ID is not authentication (base spec section 11). The signed radio policy still
accepts exactly 255/190. `wire::decode` implements it; a contract test covers 255/190 and 255/0
accepted, 255/1, 254/0 and 0/0 refused on the wired policy, and 255/0 refused on the signed one.

After: **unmodified pyflix passes 19 of 21 steps** (`pyflix-default-sender.json`), the same two
deliberate refusals as above. `pyflix-component-190.json` is the run with the sender forced to 190.

## QGroundControl v5.0.8 (2026-10-13)

Executable: `.local/qgc/QGroundControl.app` from the release DMG (GitHub digest
`sha256:1f5fc88f…74822`, verified); executable SHA-256 `1700a827…3fce0`, the same as the earlier
evidence; notarized Developer ID, `codesign --verify --deep --strict` passes.

| Check | Result |
| --- | --- |
| `tools/qgc-acceptance.py` (automated, signed per-boot session) | **pass**: vehicle discovered, signed parameter request, all 117 parameters (including the 21 `POS_*`) transferred; QGC reports initial load complete |
| `tools/qgc-session.py` (operator checklist: console `stab`, virtual joystick, arm, throttle, 3 s link cut, reconnect, re-arm, disarm) | **not completed**: QGC connected and loaded parameters, then the session ran 15 minutes with no operator input and was stopped. QGC sent only its own background requests (COMMAND_LONG 512 and 521, both answered UNSUPPORTED) |

**Finding: QGC signing depends on the host time zone.** With the machine's local zone (UTC+6) QGC
received 1,155 to 1,898 correctly signed frames per run and sent nothing back, in three runs. With
`TZ=UTC` (as the automated tool always set) it connected at once. Inferred cause: QGC derives its
MAVLink signing timestamp from local time, so outside UTC it treats a vehicle signing with the
standard UTC-based timestamp as out of its time window. Not yet checked against QGC's source. If
true, operators outside UTC would need to start QGC with `TZ=UTC` until it is fixed upstream;
it does not affect the unsigned wired path or pyflix.

Also added for the session: console `stab`, `acro`, `auto` (explicit source selection, as the
legacy console offered; QGC's generic-vehicle UI has no mode switcher), and the bench may now run
up to 15 minutes.

## Not established

- QGroundControl UI arming, virtual joystick, disarm and reconnect: the operator session is ready
  (`tools/qgc-session.py`) but has not been completed.
- The QGC time-zone cause above is inferred, not confirmed in QGC's source.
- Signed radio use by pyflix (upstream pyflix has no signing support).
- Real serial/USB, radio and hardware.
