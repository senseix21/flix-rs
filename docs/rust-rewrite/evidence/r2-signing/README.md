# R2-05: signed radio link under attack and failure

Date: 2026-10-13. Base: branch `r2/client-compat` at `b480c8a` plus the tests below.
Requirements: R2-05 ([remaining plan](../../remaining-plan.md)), SAF-03/04/05, ADR-002.
Hardware profile: none. Host only; real radio load and range belong to R5.

## In plain terms

Someone without the session key cannot make the drone do anything over the radio, cannot
replay old traffic (not even from before a reboot), and cannot steal the drone's replies.
Losing the link stops the motors, and nothing except a fresh, explicit arm starts them again.
A stop always wins, even under a flood of valid traffic.

## What runs

Both are part of `sh scripts/check.sh`, so CI runs them on every push.

| Command | What it drives |
| --- | --- |
| `cargo test -p flight-esp32 --test radio_adversarial` | `RadioSession::receive` and `enroll` with a real `Owner`, ticked at 1 kHz: the exact calls the firmware's UDP and ESP-NOW transports make |
| `sh scripts/radio-endpoint-test.sh` | the real `apps/esp32/idf/main/radio.c` built on the host against stub ESP-IDF headers, over real loopback UDP; the owner is a stand-in that accepts datagrams starting with `A` |

Toolchains: rustc 1.92.0; Apple clang 21.0.0 (macOS) and gcc in Ubuntu 24.04 (CI's runner image).

## Results

All 10 Rust tests and all 9 endpoint checks pass. Workspace total: 109 Rust tests passing.

| R2-05 item | Test | Result |
| --- | --- | --- |
| Missing key | `no_key_wrong_key_and_unsigned_frames_never_admit` | before enrollment a correctly signed arm is refused |
| Bad key, unsigned frames | same | wrong-key stab/sticks/arm/heartbeat and an unsigned arm with the right sender are all refused (5 rejections counted); the drone stays in Standby. A wired all-zero key revokes the session |
| Tampering | `every_single_bit_flip_is_refused_and_leaves_no_trace` | each of the 8 bits in every byte of a 58-byte signed arm is flipped, one at a time (464 copies), and every copy is refused. All truncations and a 1-byte extension are refused too. Afterwards the untouched original is still accepted, so no refused copy took a stream slot or moved a replay counter |
| Duplicates, reordering | `duplicate_and_older_frames_are_dropped_not_reordered` | duplicates and frames older than the newest seen on their stream are dropped, not reordered. A stop captured earlier cannot be replayed; a fresh stop works |
| Full replay-stream table | `stream_table_is_bounded_and_cannot_be_filled_without_the_key` | forged frames on all 256 link IDs take no slot. The key holder gets 4 streams; a 5th is refused and the 4 keep working |
| Restart | `restart_revokes_and_pre_reboot_traffic_stays_refused` | after a reboot, captured stab/sticks/arm/thrust frames are refused with no session, after a new-key enrollment, and **even after re-enrolling the same key**, because the new trusted time floor is later than the capture |
| Delayed traffic | `delayed_traffic_older_than_enrollment_or_sixty_seconds_is_refused` | signed for before enrollment: refused. Never seen, held back 59 s: accepted (MAVLink's 60 s window). Held back 61 s: refused |
| Unauthorized endpoint change | `radio-endpoint-test.sh` | the reply address is learned only from an accepted frame. Refused and oversize (281-byte) datagrams from another address do not move it. A new session epoch (enroll, rotate, revoke) forgets it |
| Key rotation | `rotation_needs_a_stopped_drone_and_drops_the_old_key` | armed: rotation is refused and the flight continues under the current key. Stopped: after rotation old-key frames are refused and new-key frames fly. Interrupted rotation (wrong destination): radio off, motors stopped |
| Transport loss | `transport_loss_expires_the_lease_and_reconnect_does_not_resume` | with no frames, the 1 s command lease expires: motors stop, reason `CommandExpired`. After 100 fresh, valid control frames the motors stay stopped and the drone does not re-arm |
| Admission does not renew authority | `accepted_heartbeats_and_admission_do_not_renew_control_authority` | 1.2 s of accepted heartbeats only: lease expires and motors stop |
| Stop under overload | `stop_is_not_displaced_by_a_flood_of_accepted_traffic` | 1,000 accepted idle-stick + arm frames, sent before or after a stop in the same tick: motors stop, Standby, and the queued arms are discarded rather than run later |

## The tests can fail (mutation checks)

Each code change below was applied temporarily, the suite was run, and the change was reverted.

| Mutation | Caught by |
| --- | --- |
| record the replay stream before checking the signature (`wire.rs`) | bit-flip test, stream-table test |
| skip the floor advance on receive (`runtime.rs`) | delayed-traffic test |
| rotation does not revoke first (`session.rs`) | rotation test, no-key test |
| stop does not clear queued commands and setpoints (`ingress.rs`) | stop-under-overload test |
| stop refused when the queue is full (`ingress.rs`) | stop-under-overload test |
| `radio.c` learns the endpoint from any datagram | 4 of 9 endpoint checks |

The first version of the overload test used 90% throttle. It missed the "stop keeps queued
commands" mutation, because the drone also refuses to arm above idle throttle. The flood now
uses idle sticks plus arm, so only the stop's clearing keeps the drone down. One equivalent
mutation survives: if the stop keeps queued commands but clears the setpoints, a queued arm is
still refused, because arming needs a fresh lease from the selected source.

## Behaviour worth knowing

- **Out-of-order frames are dropped, not reordered.** On a lossy link, a late packet is lost
  rather than applied out of order. Control uses latest-wins, so this is the intended trade.
- **A legitimate sender can use up to 4 link IDs.** QGC and pyflix each use one.
- **The reply address follows the key holder.** An accepted frame from a new address moves it,
  so an operator who roams keeps working. Moving it takes a fresh, correctly signed frame.
- **Same-key re-enrollment is safe only with a later trusted time.** The enrollment tool
  generates a fresh key every boot (ADR-002). The test shows that the time floor alone also
  refuses captured traffic.

## Not established

- The stand-in owner in the endpoint test is not the Rust firmware crate. That link is
  established separately: `RadioSession::receive` returns true only for admitted frames.
- ESP-NOW transport: peer filtering (`espnow.c`) and its overflow-stop path are not exercised
  on the host.
- Real radio loss, latency, interference and load (R5).
