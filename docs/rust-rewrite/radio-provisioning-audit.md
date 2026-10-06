# Radio provisioning audit — R2-01

Historical audit: this predates the current wired session flow. The exported
provisioning hook has been removed. See [ADR 002](adr-002-wired-session.md) and
the [execution report](execution-report.md) for its replacement and limits.

Date: 2026-10-05. Observed implementation, not an accepted provisioning design.
Maps to P4-04/05 and SAF-03/04/05. Hardware/client integration remains open.

## Current path

`Firmware.radio` starts as `None`. `flix_board_radio` rejects traffic until a
policy is installed. SDK UDP polling learns an endpoint only when that admission
returns success. RX handles at most four datagrams per wake.

`flix_board_provision_radio` is declared in `apps/esp32/idf/main/board.h` and
implemented in `apps/esp32/firmware/src/lib.rs`. There is no SDK caller or usable
USB enrollment command. Provisioning requires **stopped exclusive maintenance**,
a readable 32-byte key, and a trusted floor. A nonzero key/floor is validated;
its provenance is the caller's responsibility. Sender policy is currently fixed
to system 255/component 190. That must be checked against actual QGroundControl
before claiming compatibility; the client's identity must not bypass signing.

`Signing.advance` interprets the floor as MAVLink time at monotonic boot zero,
then advances with a 60-second lag allowance. An enrollment timestamp captured
later in boot cannot simply be passed as a boot-zero floor without accounting
for elapsed monotonic time. Integer conversion, 48-bit timestamp capacity and
clock correction need explicit enrollment contracts and boundary tests.

Signing replay slots are RAM state; four system/component/link streams are
tracked per policy. The current system does not establish a trusted floor across
reboot. Nominal signing tests do not prove prior-boot replay exclusion under a
real enrollment/client workflow.

Telemetry uses the unsigned encoder. The Python observer verifies CRCs but does
not authenticate telemetry signatures. Signing command admission is therefore
not complete bidirectional signing interoperability. Real-client tests must
check receive policy and decide whether signed replies/verification are needed.

## Next design experiment — R2-02

Compare two concrete candidates before implementation:

1. Trusted wired enrollment with a **fresh unpredictable key every boot** and an
   explicitly derived time baseline. Test old-key replay after reboot independent
   of the previous client's timestamp. Measure operator/QGroundControl setup
   burden; a repeated key must not be described as a new session.
2. Persistent keys with a validated replay/time mechanism across reboot. Test
   backward/forward clock corrections, interrupted persistence and previously
   captured future-dated packets. A trusted current-time floor alone does not
   prove that every previous-boot packet has a smaller timestamp.

Keep enrollment/revocation in stopped exclusive maintenance and bound every
handoff. No radio enrollment from unauthenticated discovery or unsigned network
traffic. Record secret lifetime, rotation, denied requests and failure behavior;
keys never enter console logs, artifacts or trace files.

Output: ADR and independent failure fixtures, then a runnable enrollment slice
and actual-client sessions. This audit does not enable radio or select a hardware
profile. Until the workflow is implemented and verified, radio admission remains
denied by default.
