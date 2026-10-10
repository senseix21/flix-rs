# Fuzzing untrusted-input parsers

Date: 2026-10-10. Base: commit `75dc221` on branch `hardening/unsafe-and-properties`.
Requirements: SAF-03, SAF-04, SAF-05, SAF-08; P4 bounded protocol; P5 settings records; P7 RC/proxy.
Hardware profile: none. Host libFuzzer runs on macOS arm64 (nightly Rust, cargo-fuzz 0.13.2).

## Targets (`fuzz/fuzz_targets/`)

| Target | Input | Oracle beyond "no panic" |
| --- | --- | --- |
| `wire_decode` | Raw bytes into whole-frame decode (bench and signed policy), streaming `Decoder`, `event`, `console` | Accepted packet length fits the input |
| `wire_valid` | Valid-CRC frames from `wire::encode` for every dialect ID plus one unsupported ID; optionally signed | Decoded ID/length match; a signed frame is never accepted twice |
| `sbus` | Raw SBUS bytes into `Sbus::decode`, `pilot` scaling and `Owner::receive_rc` | Every produced pilot value is finite |
| `config_record` | Raw records, plus a copy with repaired header/checksum so value validation is reached | Accepted records re-encode canonically; recovery slot is 0 or 1 |
| `proxy_bridge` | Random serial/ESP-NOW/reset/time operations against `Bridge` | Output frames fit one 250-byte ESP-NOW payload |
| `owner` | USB bytes, valid frames for every ID, radio bytes, SBUS and sensor ticks into `Owner` as the firmware bridge delivers them | Committed motors finite, in [0, 1], zero outside Armed/Maintenance |

## Results

Initial run: all six targets in parallel, 300 s each (`-max_total_time=300`). No crash, panic or
assertion failure; `fuzz/artifacts/` stayed empty.

| Target | Executions | Final coverage (cov / features) |
| --- | ---: | --- |
| wire_decode | 41,146,814 | 168 / 599 |
| wire_valid | 46,591,160 | 567 / 879 |
| sbus | 5,192,752 | 755 / 993 |
| config_record (before repair path) | 26,389,854 | 609 / 1,625 |
| proxy_bridge | 9,494,941 | 182 / 1,058 |
| owner | 2,298,798 | 1,484 / 4,755 |

After adding the repaired-record path, `sh scripts/fuzz.sh 30` exited 0, with between
241,501 (owner) and 5,199,013 (wire_valid) executions per target.

## Validating the harness

- **Reachable planted bug, caught:** removing `s.validate()?` from `flight_config::decode`
  made `config_record` panic after 2 s (decoded settings failed to re-encode).
- **Planted bug not reachable, not caught:** removing `bytes.len() > 250` from
  `flight_protocol::proxy::relayable` survived 2,837,653 executions. The bound cannot trigger:
  the largest dialect payload is 97 bytes (ID 120), so a valid signed frame is at most
  122 bytes, and `signed_envelope` rejects larger length fields before that check. It stays as
  defence in depth. The proxy firmware export rechecks the same 250-byte limit (unsafe audit).
- **Coverage gap found and fixed:** the original `config_record` target fed only raw bytes, and
  random mutations almost never produce a valid 32-bit checksum. The planted validation bug
  became reachable only after the repaired-record path was added.

## Reproduce

```sh
rustup toolchain install nightly
cargo +nightly install cargo-fuzz --version 0.13.2 --locked
sh scripts/fuzz.sh 300          # every target, 300 s each
```

CI adds a `fuzz` job running 120 s per target and uploads `fuzz/artifacts/` on failure. It has
not run remotely yet.

## Unresolved

- Coverage is libFuzzer edge coverage only; no source-level coverage report was produced.
- `wire_valid` signs with a fixed key and checks single-frame replay; reordered streams and the
  full replay window remain covered by host tests, not fuzzing.
- Nightly Clippy warns that `AtomicU32::fetch_update` is deprecated in favour of `try_update`
  (`crates/flight-core/src/safety.rs:160`). The pinned stable 1.92 toolchain does not warn.
