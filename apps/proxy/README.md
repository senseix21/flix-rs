# Flix serial / ESP-NOW proxy

A separate ESP32-S3 application bridges native USB Serial/JTAG to **one
explicitly provisioned encrypted ESP-NOW peer**. Rust owns framing, bounded
queues, expiry, and Stop priority. The small ESP-IDF adapter owns USB and radio
IO. There is no flight controller in the proxy.

## Contracts

- Raw MAVLink 2 frames, unchanged bytes, signed flag required, frozen dialect
  CRC and sender IDs checked. SETUP_SIGNING is never forwarded.
- The proxy holds no MAVLink session key and does not authenticate signatures.
  The flight owner and Python reader perform authentication and replay checks.
  PMK/LMK are separate radio encryption keys, held by the SDK.
- At most 250 bytes per radio packet, matching the existing flight adapter.
  Larger packets are rejected; no fragmentation or discovery is implemented.
- Four waiting packets in each direction; a separate Stop slot clears older
  queued flight commands. Queue residence and partial framing expire after
  100 ms. Stop priority does not bypass endpoint signature checks.
- SDK callbacks copy only known-peer packets to a four-entry static queue.
  The owner drains at most four packets per iteration. Overflow clears queues.
  One radio send may be in flight; a missing callback after 100 ms resets the
  proxy instead of reusing an ambiguous completion.
- Cable connection changes clear queues and USB buffers. USB SOF detection
  does not detect an application closing a still-connected serial port.
  The proxy never manufactures a disarm signature; the flight command lease
  remains the loss-response boundary. Already submitted radio packets cannot
  be cancelled by USB disconnect or a later Stop.

## Build

Activate ESP-IDF **v5.4.2** and the pinned `esp` Rust **1.88.0.0** toolchain:

```sh
sh scripts/build-proxy.sh
cargo check --manifest-path apps/proxy/firmware/Cargo.toml --locked --target thumbv7em-none-eabihf
```

The default image has no provisioning data and does not start forwarding.
No build script flashes a device. Native USB is kept free of application logs;
boot-ROM behavior and actual serial attachment still require physical checks.

## Pairing and session enrollment

Prepare two private JSON files (mode 600), each with exactly `peer_mac`, `pmk`,
`lmk`, and integer `channel`. MACs identify the *other* board; the 16-byte keys
are hexadecimal strings, and both boards use the same PMK/LMK and channel.
No MAC/key defaults are supplied. Configurations and images contain secrets;
NVS encryption is not enabled by this development profile.

```sh
python3 scripts/proxy-provision.py --role proxy --config .local/pair.proxy.json --output .local/pair.proxy.nvs.bin
python3 scripts/proxy-provision.py --role flight --config .local/pair.flight.proxy.json --output .local/pair.flight.proxy.nvs.bin
```

The generator writes a new private 0x6000-byte NVS image without flashing.
It must match the reviewed device partition table. Installing a whole NVS image
replaces existing settings in that partition; review exports and recovery first.
Flight-side provisioning uses namespace `flixespnow`; proxy uses `flixproxy`.

On the flight image enable `CONFIG_FLIX_ESPNOW_ENABLE=y`, keep Wi-Fi station
mode disabled, and retain reviewed sensor/output settings. After boot, use the
**direct wired flight connection** to enter stopped Maintenance. The owner
attempts saved transport provisioning once per boot there. Missing or invalid
configuration requires repair and reboot. Enroll the fresh MAVLink signing
session directly on the flight board using the existing enrollment tool, then
leave Maintenance disarmed. Session keys are never stored in proxy NVS.

Connect the signed client to the proxy port:

```python
from pyflix.client import Flix
from pyflix.serial import SerialTransport

with Flix(SerialTransport("/dev/your-proxy-port", proxy=True), key=session_key) as drone:
    drone.disarm()
```

`session_key` is the fresh key enrolled directly on the flight board. Both
boards must be paired and awake; reopening the host client does not enroll a
new flight session.

## Verification scope

`sh scripts/check.sh` builds `proxy-bench` and runs a real POSIX pseudo-terminal
through the Rust bridge to the signed Rust owner, with UDP standing in for radio.
It verifies mode selection, explicit arm/Stop, parameter replies, refusal of
proxy enrollment, late connection, and serial EOF. Rust tests exercise full
queues, Stop priority, expiry, reset, wrong peers, unsigned/enrollment/oversized
packets and corrupted CRC.

See [the integration report](../../docs/rust-rewrite/proxy-integration.md) for
actual build evidence. Physical USB, encrypted radio exchange, callback overload,
Stop latency, radio coexistence and persistence interruption remain unqualified.

SDK contracts: [USB Serial/JTAG header](https://github.com/espressif/esp-idf/blob/v5.4.2/components/esp_driver_usb_serial_jtag/include/driver/usb_serial_jtag.h),
[ESP-NOW header](https://github.com/espressif/esp-idf/blob/v5.4.2/components/esp_wifi/include/esp_now.h).
