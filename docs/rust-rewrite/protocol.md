# Protocol contract

Only MAVLink 2 is accepted. Header flags, exact frame length, dialect CRC, source identity, destination, masks, scalar bounds, channel/index/count limits and quaternion norm are checked before any state mutation. MAVLink 2 trailing-zero truncation is supported by initialized padding. Unsupported messages/commands are denied; valid unsupported COMMAND_LONG requests receive an UNSUPPORTED ACK. All encoders initialize every field.

The frozen subset includes heartbeat, parameters, manual controls, arm/disarm and mode commands, attitude/rate targets, allowed console operations, versioned log transfer and telemetry. `protocol/common-subset.json` records exact CRC/min/max length entries and the SHA-256 of the official downloaded common table. Regenerate/check offline with `scripts/generate.py`. Source selection is explicit; mode 1=ACRO, 2=STAB and 3=AUTO. Source transitions stop and require rearming. Direct actuator and position/velocity flight commands are unsupported.

Bench UART permits only sender system 255/component 190 without signing. This development policy is never used by the firmware radio path. Radio denies all commands until trusted wired maintenance enrollment installs a fresh 32-byte session key and a trustworthy MAVLink timestamp anchored to the current monotonic time. Signed frames check the SHA-256 signature, per-system/component/link strictly increasing timestamps, four-stream capacity, and an advancing local floor with the protocol's 60-second lag allowance. Reboot starts unprovisioned; a fixed build constant or recycled session key is insufficient. See [ADR 002](adr-002-wired-session.md). Validate this with actual clients before wireless flight.

UDP endpoint learning occurs only after decoding, authorization and typed validation succeed. RX handles at most four datagrams per owner wake. ESP-NOW uses explicit encrypted unicast provisioning, four peers, 250-byte frames and a static four-packet callback queue; overflow invokes stopped overload behavior. Unsolicited discovery is rejected. SDK callbacks never invoke the Rust owner.

The simulator host ABI and loopback bench runner explicitly permit unsigned simulation traffic. Their unauthenticated policy must not be reused for board firmware. Python network control requires signing unless `simulation=True` is explicitly used on 127.0.0.1. The Python client validates incoming signatures/source/replay when a session key is supplied; unsigned telemetry is not authenticated state evidence.

## Frozen timing snapshot

`metrics <eight hexadecimal digits>` is a typed SERIAL_CONTROL console operation.
It is accepted only in stopped exclusive maintenance without a diagnostic permit.
The digits are a request nonce, not a credential. A snapshot freezes 52 `u64`
values: version 1; five histograms of count, maximum and eight buckets; invalid
sample-age count. Histogram order is acquisition, sample age, whole owner tick,
acquisition plus owner, then schedule jitter. Counters are cumulative for the
owner object's lifetime and saturate. They do not measure physical motor stop.

Each SERIAL_CONTROL response uses device 10 and initialized padding. Its data is
`FLXM` (four bytes), nonce (`u32` little endian), chunk index (`u8`), total chunks
(`u8`, eight), then up to seven little-endian `u64` values. The last chunk contains
three values. Denial is the ten-byte header with total zero. At most one chunk is
queued per tick and the cursor waits for reply capacity. Stop, reset, enrollment
rotation, or leaving exclusive maintenance cancels the remaining transfer.

The Python client registers its waiter before sending and permits one snapshot
at a time. It correlates the nonce, handles reordered/identical duplicate chunks,
and denies conflicting, malformed or incomplete snapshots. This response framing
is a Flix development extension inside standard SERIAL_CONTROL, not a promised
QGroundControl command.

Default telemetry: heartbeat/battery 2 Hz, attitude quaternion 20 Hz, RC/motor controls 10 Hz, scaled IMU 15 Hz. Timestamps use milliseconds for attitude/IMU/RC and microseconds for actuator controls. FRD/NED signs are converted at the boundary. Battery measurements remain unknown until an actual battery adapter/policy is qualified; all unknown sentinel fields are initialized explicitly.

## Provisioned serial proxy

The [proxy application](../../apps/proxy/README.md) forwards unchanged signed
MAVLink packets with a 250-byte radio limit. It rejects enrollment, checks
framing/CRC/source, and reserves Stop capacity; only the receiving owner/client
checks signatures and replay. Pairing encryption keys are installed offline in
private role-specific NVS. MAVLink enrollment still terminates on the actual
flight board's direct wired interface, once per boot. The host pseudo-terminal
integration and SDK builds do not establish encrypted radio qualification.

## QGroundControl discovery and signing

AUTOPILOT_VERSION (148) answers REQUEST_MESSAGE (512, parameter 1 = 148)
and REQUEST_AUTOPILOT_CAPABILITIES (520, parameter 1 = 1). Advertised capabilities
are MAVLink 2, float parameters with C-cast encoding, and SET_ATTITUDE_TARGET.
Mission upload, GPS navigation and hardware identity are not advertised.
PROTOCOL_VERSION (300) answers REQUEST_MESSAGE (512, parameter 1 = 300)
and REQUEST_PROTOCOL_VERSION (519, parameter 1 = 1), reporting current/minimum/
maximum version 200. This prevents QGC's initial negotiation falling back to
MAVLink 1, which cannot carry signed commands and is rejected by this endpoint.
Heartbeat custom mode reports ACRO=1, STAB=2, AUTO=3.

QGroundControl v5.0.8 hashes its signing passphrase with SHA-256. Generate a fresh
private phrase using `tools/enroll-radio.py --qgc-passphrase-file PRIVATE_FILE`
along with the direct wired device and session-file options. Do not reuse a phrase
after reboot. This session key is distinct from the peer PMK/LMK.

The v5.0.8 signing epoch is initialized from local midnight on 2015-01-01.
In a UTC+6 process this differs from the standard UTC epoch. Launch that QGC
executable with `TZ=UTC` for interoperability; this changes only the process
and does not change the operating system timezone. Link classification can also
skip signing on connections QGC considers secure. Verify actual signed traffic;
entering a key in settings does not alone prove a signed session.

`PYTHONPATH=tools python3 tools/qgc-acceptance.py --executable QGC_BINARY --output NEW_DIRECTORY`
uses private XDG settings, UTC, a fresh host-only enrollment, a loopback observer,
and the real Rust owner. It records headers and parameter indices without keys.
`--trusted-local` selects the explicitly unsigned host-only bench policy; it does
not alter production radio admission. Actual-client results and remaining UI
limits are in [software acceptance](software-acceptance.md).
