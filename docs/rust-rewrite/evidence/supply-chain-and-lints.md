# Supply-chain policy and lint hardening

Date: 2026-10-10. Base: commit `b573b11` on branch `hardening/unsafe-and-properties`.
Requirements: P8-06 release provenance; `dependency-policy.md` advisory and licence review.
Hardware profile: none.

## cargo-deny

`deny.toml` (cargo-deny 0.20.2) and `scripts/deny.sh` check all four locked workspaces: host,
ESP32 firmware, proxy firmware and fuzz. Result on 2026-10-10: every workspace reports
`advisories ok, bans ok, licenses ok, sources ok`.

- **Advisories:** any RustSec vulnerability, unsound or yanked crate fails. The check needs
  network access for the database, so it runs in its own CI job (`supply-chain`), not in the
  offline `scripts/check.sh`.
- **Licences:** the allow-list is MIT and Apache-2.0. Two crate-scoped exceptions, neither
  linked into firmware: `libfuzzer-sys` (NCSA; bundles LLVM libFuzzer source, fuzz workspace
  only) and `unicode-ident` (Unicode-3.0; proc-macro build dependency of the fuzz workspace).
- **Sources:** crates.io only. No unknown registries or git sources.
- **Project crates:** now `publish = false` and skipped by the licence check, because no
  project licence has been chosen. This must be revisited when a licence is added.

## Panic lints in the portable core

`flight-core`, `flight-protocol` and `flight-config` now deny `unwrap_used`, `expect_used`,
`panic`, `todo`, `unimplemented`, `unreachable`, `dbg_macro`, `print_stdout` and
`print_stderr`.

- Removed: `expect` in `flight_protocol::f32_at` and `flight_config::decode`, replaced by direct
  indexing of the fixed 255-byte payload and fixed 404-byte record, as `i16_at` already did.
- Kept as `#[expect(clippy::expect_used, reason = ...)]` (a stale exception becomes a warning):
  `RingLog::encode` (bounded record index), `Flight::default` (panics only after `u32::MAX`
  owners exhaust issuer IDs), `Settings::tuning` and `Settings::limits` (generated schema keys).

## Measured backlog, not yet enforced

Clippy findings on the three core crates (lib targets, 2026-10-10), counted per lint including
the summary line:

| Lint | Approx. findings | Why it matters |
| --- | ---: | --- |
| `indexing_slicing` | 144 | Each is a possible panic; the parser ones are covered by fuzzing |
| `arithmetic_side_effects` | 93 | Overflow in time and size arithmetic |
| `as_conversions` | 36 | Silent truncation or saturation |
| `cast_possible_truncation` | 23 | e.g. `(seconds * 1e6) as u64` for leases |
| `float_cmp` | 10 | Exact float equality, e.g. Stop detection in `relayable` |
| `cast_sign_loss` | 9 | Negative float to unsigned |
| `missing_docs` | 333 | Public API documentation |

These should be fixed per module with focused tests, not by bulk allow-listing. Casts and float
comparisons in the safety and protocol paths come first.

## Cast and float-comparison review (follow-up)

The core crates now also deny `cast_possible_truncation`, `cast_sign_loss`,
`cast_possible_wrap` and `float_cmp`. All 38 findings were reviewed; none was a live defect
under current invariants.

- **Removed by construction:** `wire::crc` and `wire::encode` take bytes via `to_le_bytes` and
  `u8::try_from`; the record header count uses `u16::try_from(COUNT)`.
- **Telemetry helpers:** `boot_ms` (MAVLink `time_boot_ms` wraps after about 49.7 days by
  definition) and `saturating_i16`.
- **Finding:** `SCALED_IMU` reports gyro in mrad/s as i16. The MPU6050's ±2000 deg/s range
  (34.9 rad/s) exceeds i16, so gyro above about 1,877 deg/s saturates in telemetry. Control
  is unaffected. Resolved by adding HIGHRES_IMU alongside it; see [highres-imu](highres-imu.md).
- **Reasoned `#[expect]`:** `flight_protocol::event` (MAVLink integer arguments are exact
  floats; a tolerance would accept 0.5 as Stop; every cast follows a range check),
  `Settings::scalar`, `validate`, `limits` and `telemetry::battery_voltage`.
- **New `crates/flight-config/tests/schema_invariants.rs`:** checks the schema facts those
  reasons depend on. Integer parameters stay within ±2^24, `MOT_PWM_RES` <= 31 (shift),
  `SF_RC_LOSS_TIME` fits microseconds, and defaults supply every tuning and limit key.

Verification: `scripts/check.sh` exit 0 (54 Rust tests, 34 Python, 400 parity cases); both
Xtensa archives build; `scripts/fuzz.sh 20` exit 0 for all six targets.

## Verification

`scripts/check.sh` exit 0 (50 Rust tests, 34 Python tests, 400 parity cases, max error 4.77e-7);
`scripts/deny.sh` passes; both Xtensa archives build; all six fuzz targets build.
