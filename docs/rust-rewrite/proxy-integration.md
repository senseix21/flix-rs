# Provisioned USB / ESP-NOW proxy integration

Date: 2026-10-06. Local uncommitted development; no qualified radio or flight release.
This continues [Phase 6 adverse integration](phase6-adverse-integration.md).

## Delivered software

- Added `flight-proxy`, an allocation-free one-owner bridge, its real POSIX host
  executable, and a separate ESP32-S3 Rust static library/ESP-IDF application.
- Defined one provisioned peer, raw signed MAVLink framing, 250-byte radio
  capacity, four waiting packets in each direction, separate Stop capacity,
  100 ms residence/partial-frame expiry, reset, and explicit overflow handling.
- Preserve bytes and endpoint signatures. The transparent bridge checks framing,
  CRC and source but holds no MAVLink session key. Signature/replay admission
  remains on the flight owner and Python reader. Enrollment packets are blocked.
  The older `proxy::admit` helper remains available for an authenticated relay;
  it is not the transparent application's policy.
- Added private offline NVS generation for both ends. Proxy reads `flixproxy`;
  flight reads `flixespnow` only after explicit stopped Maintenance, once per boot.
  Transport keys persist separately from the ephemeral MAVLink session.
- Added disabled-by-default `FLIX_ESPNOW_ENABLE`, separate from the station Wi-Fi
  profile. The existing four-peer driver still supports its lower-level API;
  this saved-pairing workflow deliberately provisions one peer.
- Reject unknown peers before callback queue capacity is consumed and protect
  shared peer membership with a critical section. Callbacks never call Rust.
- Fixed the logical-clock bench's late-client ACK loss by draining unroutable
  telemetry, matching the previously corrected Gazebo adapter.

The first serial integration attempt failed waiting for the first mode ACK;
its log is retained. A forced 100 ms late connection now exercises the fixed
bench. An early oversized-packet unit fixture referenced a message outside the
frozen dialect; the fixture was corrected to construct a bounded malformed
280-byte envelope rather than expand the protocol.

## Contracts and operation

[Proxy README](../../apps/proxy/README.md) describes exact build, pairing,
capacity, disconnect, key storage, and serial usage contracts. No image contains
a default peer/key. No script flashes a device. No motor mapping or controller
gain was changed.

One in-flight radio packet cannot be cancelled by Stop or cable loss. The bridge
does not synthesize signed disarm packets. Application-close detection differs
from cable SOF detection; the flight command lease remains the loss boundary.
Native USB and radio callbacks need actual physical timing and overload checks.
Transport NVS contains secrets and this development profile does not encrypt
NVS; generation enforces private file permissions and never prints key values.

## Verification

Detailed commands, logs, configuration/source fingerprints, and artifact hashes
are in [the evidence summary](evidence/proxy-integration/summary.json).
Host integration uses the real Python serial client, a POSIX pseudo-terminal,
the real Rust bridge, and the signed Rust owner; loopback UDP stands in for radio.
It checks mode/arm/Stop, parameter replies, forbidden proxy enrollment, bad keys,
late connection, and serial disconnect. This is not encrypted radio evidence.

The host suite includes 45 Rust tests, 30 Python tests, 400 pinned C++ scalar
comparisons and 14 ABI scenarios. Separate `no_std` and actual Xtensa archive
checks cover the proxy firmware. SDK link results are recorded in the summary.

| ESP32-S3 image | Bytes | SDK result |
| --- | ---: | --- |
| Proxy | 696,192 | Pass; 34% of the 1 MiB app partition free |
| Default flight | 440,464 | Pass; 58% free |
| ESP-NOW flight | 939,712 | Pass; 10% free |

All flight builds retain disabled PWM and unassigned sensor pins; the optional
profile enables only ESP-NOW. The development artifact set in
`.local/proxy-delivery-20261006/` includes each app, bootloader, partition table,
SDK configuration and flash metadata with verified hashes; private NVS images
are excluded. No artifact was flashed.

The serial host adapter takes ownership of the inherited pseudo-terminal fd
rather than reopening `/dev/fd`, which can create a new master on Linux. The
final host run exercises this ownership path on macOS; remote Linux CI has not
run, and no Linux execution is claimed.

Gazebo was not rerun for this transport-only pass. Its seventeen-scenario result
belongs to the preceding evidence bundle. Docker was unavailable in this session;
no live simulator was started or restarted.

## Remaining gates

P7-05/R7 transport work now has runnable software and host integration. Physical
USB attachment, paired encrypted exchanges, peer/key persistence interruptions,
callback overload and Stop latency, RC/radio coexistence, instrumented workload,
QGroundControl compatibility and hardware flight qualification remain open.
P7-03 through P7-06 are not marked complete by compile or pseudo-terminal tests.

SDK references: [pinned USB driver](https://github.com/espressif/esp-idf/blob/v5.4.2/components/esp_driver_usb_serial_jtag/include/driver/usb_serial_jtag.h),
[pinned ESP-NOW API](https://github.com/espressif/esp-idf/blob/v5.4.2/components/esp_wifi/include/esp_now.h).
