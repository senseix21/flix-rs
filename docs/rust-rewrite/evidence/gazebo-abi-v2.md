# Gazebo host ABI v2: IMU range-limit mask

Date: 2026-10-10. Base: commit `1577a81` on branch `hardening/unsafe-and-properties`.
Requirements: P6-02/P6-05 simulator parity of sensor handling; SAF-03 sensor evidence.

## Change

- `FlixInput2` (56 bytes) appends `saturation` and `reserved` (offsets 48 and 52) to the
  48-byte version 1 input. A separate `flix_tick2` export takes it, so a version 1 caller
  never has 56 bytes read from a 48-byte object.
- Both exports validate their own pointers, then share one safe tick body. `flix_tick2`
  rejects version != 2, mask bits above 5 and a nonzero reserved word with -4 and stopped
  output.
- The plugin sends `sensor_limit_mask`, the same mask it already writes to `FLXT` v3 traces,
  through `flix_tick2`. The owner's `record_imu_saturation` turns it into counters and the
  rate-limited STATUSTEXT warning, as on hardware.
- `tools/gazebo-scenario.py` now reads owner replies on the client socket. With the Rust
  engine, any scenario with range-limited ticks fails unless an `IMU saturated:` notice
  arrives. Notices are stored in `result.json`.

## Verification

| Check | Result |
| --- | --- |
| Rust ABI tests: v2 size 56, saturation offset 48; bad mask, reserved or version give -4 with zero motors; null pointer gives -3 | Pass |
| Rust ABI test: an accel z mask on one `flix_tick2` produces exactly one `IMU saturated: az` reply | Pass |
| Independent ctypes layout (`scripts/abi-smoke.py`): v1 and v2 sizes and offsets, v2 mask rejection | Pass |
| `scripts/check.sh` | Exit 0: 68 Rust, 35 Python, 400 parity cases |
| C++ plugin build against the new header (Gazebo 11.10.2, Ubuntu 22.04 amd64 container, CMake) | `flix_rust` and `flix-world-reset` targets built |
| Gazebo `landing-contact` scenario | **Not run: blocked by the environment** (see below) |

## Environment limitation

This host is Apple Silicon with Colima using QEMU for amd64. Ubuntu 22.04 has no arm64 Gazebo 11
packages, so the image is amd64 (built by running the Dockerfile's steps in an amd64 container and
committing; the classic builder resolved the cached arm64 base despite `--platform`). Under QEMU,
stock `gzserver` aborts with `std::out_of_range` while loading even
`/usr/share/gazebo-11/worlds/empty.world`, with no Flix plugin loaded. The landing scenario therefore
recorded zero ticks. This is an emulation failure, not a result for this change.

The earlier recorded Gazebo evidence was x86_64. Re-running needs native amd64 (the CI `gazebo`
job on `ubuntu-24.04`) or a Rosetta-backed VM. Switching this Colima profile to Rosetta would
recreate the VM and was not done.

## Unresolved

- Gazebo landing-contact, saturation-recovery and axis-pulse scenarios with the v2 plugin,
  including the new operator-notice check.
