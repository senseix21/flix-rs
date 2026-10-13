# R2-04: real client compatibility (upstream pyflix)

Date: 2026-10-13. Base: `main` `fdcb651` plus the changes on branch `r2/client-compat`.
Requirements: R2-04 ([remaining plan](../../remaining-plan.md)), base spec section 11 (clear
denials, no concealed rejection). Hardware profile: none (host bench, loopback UDP).

## In plain terms

The original Flix Python client (pyflix) can now control the Rust firmware on the wired
development link: connect, read and change settings, use the console, change mode, arm, fly by
stick commands and disarm. One thing it still needs: it must identify itself as component 190
(one line, below). QGroundControl is not covered here yet.

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

## Open: sender identity

With pyflix's default sender (255, component 0 from pymavlink) the bench refuses every command
(`pyflix-default-sender.json`: `admitted={}`, 55 frames rejected; only telemetry-dependent steps
pass). The wired policy accepts exactly 255/190 and rejects component 0 on purpose. Two options:

1. Keep the policy; document `drone.mavlink.srcComponent = 190` after connecting (or use
   the repo client). No firmware change.
2. Also accept 255/0 on the unsigned wired policy only. On that path the sender ID is not
   authentication (base spec section 11), so this widens hygiene, not authority; the signed radio
   policy stays fixed.

Not decided here. ADR 002 says broader sender policy needs explicit interop evidence; this is
that evidence.

## Not established

- QGroundControl beyond the earlier signed discovery and parameter transfer: manual UI arming,
  mode change, virtual joystick, disarm and reconnect still need a QGC session with an operator.
- Signed radio use by pyflix (upstream pyflix has no signing support).
- Real serial/USB, radio and hardware.
