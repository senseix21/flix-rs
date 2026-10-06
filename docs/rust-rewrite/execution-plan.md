# Remaining implementation execution plan

Date: 2026-10-05. Applies to the authorized local implementation in `flix-rust`.
This document expands R0–R7; it does not change the meaning of their physical gates.
Method: state the invariant, trace the mechanism, implement a bounded slice,
check its failure contract, then run the consolidated suite after integration.
This applies engineering habits associated with Jon Gjengset's teaching, not his
personal advice or an organizational architecture.

## Decisions for this delivery

* Retain the audited C ESP-IDF adapter and C++ Gazebo boundary. Replacing the SDK
  runtime is a separate migration and would invalidate existing build evidence.
* Keep the original controller gains and scalar oracle. Treat the previously
  observed command-dispatch difference as a documented compatibility difference;
  do not change safety ordering to make legacy motor traces match.
* Provision signed radio through trusted wired maintenance, with a fresh random
  session key per enrollment/boot. No persistent key or unauthenticated time sync.
  USB is a physically trusted interface; radio cannot provision itself.
* Battery policy is an opt-in bench stop/arm-inhibit mechanism with timestamped
  calibrated readings. Default remains unavailable/disabled. No invented divider,
  pin, battery threshold or claimed controlled landing.
* Physical acquisition, power, motor stop, timing, actual radio coexistence and
  operator flight cannot be simulated into completion. Provide tools/procedures
  and leave those gates awaiting the exact hardware.

## Ordered executable slices

| Slice | Implementation and files | Acceptance | Dependencies |
| --- | --- | --- | --- |
| R0 | Numeric workload fixture, requirement/evidence inventory, source manifest | Bounded configured rates and explicit instrumented vs physical claims | Existing schema, ingress capacities |
| R1 | Finite Gazebo mode/lease/reconnect scenarios, reset findings and intentional parity classification | Applied motors stop on loss; reconnect never arms; output bounds and complete traces | Existing binary trace/collector |
| R2 | Trusted wired enrollment, explicit revoke/rotation, bidirectional signing, authenticated Python reader, serial enrollment tool | Armed/diagnostic enrollment denied; unsigned/bad/replayed radio denied; old-session key denied; no secret trace | MAVLink SETUP_SIGNING inventory, stopped maintenance |
| R3 | Machine-readable exact-board record and validator, compile-only defaults retained | Missing/overlapping/unreviewed pins and incomplete electrical profile denied; no automatic flashing | Delivered hardware remains external prerequisite |
| R4 | Portable battery reading/policy, owner integration, telemetry, opt-in configuration | Invalid/stale/unknown arm denial when enabled; critical voltage latches stop; no silent recovery | Accepted real scaling/thresholds required before hardware enable |
| R5 | Bounded timing distributions, acquisition/sample-age counters, bench evidence validator | Deterministic buckets/counters; missing or failed physical captures never labeled qualified | Existing owner and exact hardware profile |
| R7 | POSIX serial transport, bounded raw MAVLink framing for USB proxy integration | Partial/oversized packets bounded; partial writes handled; disconnect wakes waiters; enrollment packets excluded from wireless forwarding | Existing Python client and provisioned ESP-NOW adapter |
| R6 | Development bundle manifest, fresh build/rollback runbook, final acceptance report | Source/config hashes saved; qualification status truthful; secrets excluded | Consolidated host/no_std/target/simulator checks |

## Detailed invariants and limits

### Enrollment and signing

The owner task alone changes signing state. Parse enrollment only on the wired
path after CRC/sender/destination checks; never enqueue its key in general logs.
Entering enrollment requires stopped exclusive Maintenance with no diagnostic
permit. Rotation discards queued authority and old replay streams. A malformed
enrollment revokes radio instead of preserving ambiguous state. Reboot starts
unprovisioned. A private host file uses mode 0600 and is created exclusively;
never print its key or permit enrollment over a network/proxy transport.

Use trusted MAVLink timestamp at enrollment and a local monotonic anchor rather
than assuming enrollment occurs at boot zero. Reject timestamps outside 48 bits
and exhaustion. Sign outgoing radio telemetry with a distinct link ID and strictly
increasing timestamp. The Python client validates signature, source and replay
before satisfying requests. Wired/simulator observers retain explicit unsigned
development behavior. Actual QGroundControl compatibility remains a separate
client gate; implementing standard framing alone does not establish it.

### Battery

Validated finite voltage plus capture time is distinct from unavailable/invalid.
The policy owns debounce/hysteresis state. Fresh healthy voltage is required to
arm only when the policy is explicitly installed in stopped maintenance. Missing,
stale or critical readings stop an armed bench profile through the same Flight
fault/output gate. Policy reset cannot resurrect a cached frame. Hardware ADC
scaling and in-flight loss response require measured and accepted configuration.

### Timing and transports

Record counters with fixed memory and saturating arithmetic. Name the measured
intervals: acquisition start/end, sample age at owner tick, core duration, owner
duration and owner schedule jitter. Histograms are approximate distributions,
not raw physical timing proof. UART and callback work remain bounded. The proxy
forwards complete packets and never interprets flight lifecycle; enrolled radio
still performs authorization. USB key provisioning must terminate on the actual
flight board, not pass through the ESP-NOW proxy.

## End-of-delivery verification

1. Focused failure checks at each dependent boundary.
2. `sh scripts/check.sh`: generation, formatting, Clippy, Rust/Python tests,
   release build, scalar oracle, ABI fault replay and host benchmark.
3. Portable `no_std` check and exact Xtensa archive/SDK links where local pinned
   toolchain is present. Never use blanket embedded/host all-features.
4. Actual isolated Gazebo scenarios; preserve failures with raw traces.
5. Validate documentation links, evidence hashes and development release manifest.
6. Record physical, actual-client and operator prerequisites without marking them
   passed. No deployment, hardware upload or publishing is implied.
