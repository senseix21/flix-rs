# Why the C++/Rust plant comparison failed

Date: 2026-10-10. Branch `investigate/plant-parity`. Platform for new runs: local arm64 Gazebo
11.15.1 (development image `.local/gazebo-arm64`, not the CI evidence image). The recorded failing
pair is software-five (x86 Gazebo 11.10.2). Scenarios: `pitch-pulse`, `level-thrust`. Limits
unchanged: 5°, 0.1 motor, 0.5 m. Requirements: P6-02, R1-02. No hardware.

## Summary

The recorded C++/Rust failure (pitch: 1.05°, 0.51 motor, 0.90 m) has two independent causes. Neither
is a controller or gain difference.

1. **Simulated sensor model mismatch (attitude and position).** The legacy Gazebo plugin passes the
   accelerometer through a 0.1 first-order filter, its stand-in for the MPU6050's on-chip DLPF. The
   Rust adapter models the ±4 g range but no filter. On the ground, Gazebo contact chatter rocks the
   65 g body ±0.018° every 1 ms step (±0.32 rad/s, about 8 m/s² lateral; 180× and 90× the
   configured sensor noise). Unfiltered, |acc| ≈ 13 m/s², so the estimator's landed gate
   (|acc| within 10% of g) passes on only 0.6–16% of resting ticks, against 99.2% for C++. The Rust
   estimate then integrates mis-sampled chatter gyro and random-walks (0.07–0.89° at arming, varying
   by run). STAB holds the *estimate* level, so the drone flies tilted by that error, and the
   tilt becomes lateral drift.
2. **Command dispatch order (motor, isolated ticks).** All remaining violations fall on logged
   MANUAL_CONTROL transition ticks, where legacy applies a new stick value one tick later than Rust
   (legacy processes MAVLink after control; Rust applies inputs before control, a recorded safety
   correction).

## Evidence

**Physics is deterministic; sensing is not.** In the recorded pair, true pose and position are
identical on every tick until 3 ms after arming, yet resting gyro samples match on 1–2% of ticks.
In both engines the gyro agrees with truth one step later only 83–88% of the time; the rest come
from an adjacent step (the plugin reads Gazebo's IMU sensor, updated on its own thread).

**Noise floor.** Two identical Rust runs (`pitch-self.json`): 0.061°, 0.0023 motor, 0.049 m (pass;
gyro streams identical on only 3.2% of resting ticks). The simulator's own variation is 10–20×
below the limits, so the recorded C++/Rust gap was a real difference.

**Decisive experiment.** An opt-in IMU profile `mpu6050-4g-lpf01` adds the legacy 0.1 filter
(starting from zero, as legacy does) in front of the Rust adapter's range limit. Same-platform pairs:

| Scenario | Rust profile | Attitude | Motor | Position | Rust estimate error at arm (C++ 0.034°) |
| --- | --- | ---: | ---: | ---: | ---: |
| pitch-pulse | `mpu6050-4g` (default) | 0.232° | 0.504 | 0.129 m | 0.215° |
| pitch-pulse | `mpu6050-4g-lpf01` | 0.234° | 0.508 | **0.032 m** | **0.000°** |
| level-thrust | `mpu6050-4g` (default) | 0.687° | 0.452 | 0.370 m | 0.311° |
| level-thrust | `mpu6050-4g-lpf01` | **0.146°** | 0.465 | **0.058 m** | **0.038°** |

Every violation is on a logged transition tick (pitch: 2, 1002, 1162; level: 502), classified
`explained` with no unexpected ticks. Excluding those ticks, the stable-interval gate passes in all
four pairs (maximum motor difference 0.062–0.067).

## Decisions (user-approved 2026-10-10) and follow-up

**Realistic sensor model, measured first.** I added the firmware's own configuration as
`mpu6050-4g-dlpf1`: a first-order approximation of DLPF_CFG=1 (184 Hz accel, 188 Hz gyro). The
resting chatter is at 500 Hz, above that bandwidth, so the filter leaves about 4 m/s² of it:

| Scenario | Profile | Landed gate passes | Estimate error at arm | Position diff |
| --- | --- | ---: | ---: | ---: |
| pitch-pulse | default | 0.9% | 0.215° | 0.129 m |
| pitch-pulse | dlpf1 | 36.2% | 0.208° | 0.236 m |
| pitch-pulse | lpf01 | 100% | 0.000° | 0.032 m |
| level-thrust | default | 0.5% | 0.311° | 0.370 m |
| level-thrust | dlpf1 | 36.9% | 0.194° | 0.143 m |
| level-thrust | lpf01 | 100% | 0.038° | 0.058 m |

Modelling the real sensor does not reproduce the legacy simulator, which filters far more than the
hardware does. The comparison therefore uses `mpu6050-4g-lpf01` on the Rust side (recorded in
`fixtures/simulator/acceptance.json` as `comparison.rust_imu_profile`), so both controllers receive
the same sensor model. **The default simulated sensor is unchanged**; whether to switch it to
`dlpf1` is a separate decision, and the real artifact is the contact chatter.

**Acceptance rule.** `comparison.acceptance_rule` = stable-interval. `tools/gazebo-compare.py`
reports `acceptance` and exits non-zero only on acceptance failure; full-tick parity is still
reported. The rule passes all six arm64 pairs, and still **fails** the recorded x86 pair that
drifted 0.90 m (765 unexpected ticks), so it does not hide real divergence. The default profile
passed on arm64 only because its pre-arm drift happened to be small in that run.

**CI.** `make sim-parity` (in the Gazebo job) builds the pinned legacy plugin (SDL2 added to the
pinned image) and runs the pairs on x86 with the `lpf01` profile.

## x86 CI results (pinned image, Gazebo 11.10.2)

`make sim-parity` in the Gazebo CI job, Rust side `mpu6050-4g-lpf01`, rule stable-interval:

| Run | Commit | Scenario | Acceptance | Attitude | Motor (stable) | Position | Unexpected ticks |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| 38076118073 | `6ca8165` | pitch-pulse | pass | 0.236° | 0.064 | 0.048 m | 0 |
| 38089194412 | `c285593` | pitch-pulse | pass | 0.373° | 0.065 | 0.321 m | 0 |
| 38089194412 | `c285593` | level-thrust | pass | 0.546° | 0.073 | 0.330 m | 0 |

(Run 38076118073 stopped after pitch-pulse on a Makefile bug, since fixed; level-thrust was not
reported there.)

**Margin finding.** Position difference varies 0.05–0.33 m between x86 runs of the same commit
(limit 0.5 m), much more than on arm64 (0.03–0.06 m). Even with matched sensor filtering, the
uncorrected estimators still diverge between engines by an amount that depends on the run's resting
chatter. Its magnitude has not been characterized statistically. With n=2, the gate may be flaky; a
repeated-run distribution (e.g. 10 pairs) is needed before treating 0.5 m as a reliable gate.

## Run-to-run spread and the contact fix (user-approved 2026-10-11)

**Rigid contact is flaky.** Ten x86 CI pairs (workflow run 38098993941; 9 of 10 artifacts
retrieved, files in `rigid-spread/`), Rust `lpf01`:

| Scenario | Pass | Position min / median / max | Max attitude |
| --- | ---: | ---: | ---: |
| pitch-pulse | 8/9 | 0.007 / 0.111 / 0.528 m | 0.68° |
| level-thrust | 8/9 | 0.037 / 0.189 / 0.538 m | 0.98° |

About one run in five would fail CI on noise.

**Soft contact removes the cause.** `scripts/gazebo-contact.py soft` gives the model's collision a
critically damped 30 Hz contact derived from the 65 g mass (kp 2309 N/m, kd 24.5 N·s/m, 0.28 mm
sag), applied to both engines' model copies. Local arm64, default (unfiltered) Rust sensor:

| | Rigid | Soft |
| --- | ---: | ---: |
| Resting lateral acceleration | 8.6 m/s² | 0.10 m/s² |
| Landed gate passes | 0.5–0.9% | 100% |
| Position vs C++ | 0.13–0.37 m | 0.004–0.007 m |
| Attitude vs C++ | 0.23–0.69° | 0.03° |

Ten x86 CI pairs with soft contact (workflow run on `ac50879`, files in `soft-spread/`):

| Scenario | Pass | Position min / median / max | Max attitude | Max motor (stable) |
| --- | ---: | ---: | ---: | ---: |
| pitch-pulse | 10/10 | 0.0050 / 0.0076 / 0.0175 m | 0.051° | 0.033 |
| level-thrust | 10/10 | 0.0044 / 0.0085 / 0.0144 m | 0.033° | 0.006 |

The worst case uses 3.5% of the position limit. `make sim-parity` now uses soft contact for both
engines and the default Rust sensor (`acceptance.json`: `plant_contact: soft`,
`rust_imu_profile: mpu6050-4g`). The `lpf01` and `dlpf1` profiles stay available but are no longer
needed for the comparison.

**Scope.** Superseded on 2026-10-11: soft contact is now the default plant (next section).

## Soft contact as the default plant (user decision 2026-10-11)

Every launcher (scenarios, smoke check, browser session, legacy comparison) now applies
`scripts/gazebo-contact.py soft` unless `FLIX_CONTACT=rigid`. Evidence recorded before 2026-10-11 used
the rigid plant; reproduce it with `FLIX_CONTACT=rigid`.

Re-baseline: every CI scenario on x86, rigid (run 38025743862) against soft (run 38103224480; raw
values in `soft-default-rebaseline.json`; `two-models` values are per model):

| Scenario | Status | Max tilt (°) rigid → soft | Max lift (m) | Range-limited ticks | Stop latency (sim µs) |
| --- | --- | --- | --- | --- | --- |
| acro-level | pass → pass | 11.59 → 11.63 | 0.14 → 0.14 | 8 → 10 | 0 → 0 |
| auto-level | pass → pass | 3.65 → 3.81 | 0.27 → 0.26 | 1 → 4 | 0 → 0 |
| axis-pulses | pass → pass | 3.67 → 3.82 | 1.25 → 1.25 | 1 → 5 | 0 → 0 |
| diagnostic-stop | pass → pass | 0.00 → 0.00 | 0.00 → 0.00 | 0 → 0 | 0 → 0 |
| landing-contact | pass → pass | 11.80 → 11.51 | 0.08 → 0.12 | 9 → 13 | 0 → 0 |
| late-client | pass → pass | — → — | — → — | — → — | — → — |
| lease-loss-reconnect | pass → pass | 3.73 → 3.82 | 0.47 → 0.47 | 1 → 5 | 0 → 0 |
| level-thrust | pass → pass | 4.95 → 4.19 | 5.09 → 5.10 | 1 → 6 | 0 → 0 |
| long-level | pass → pass | 3.79 → 3.82 | 16.35 → 16.33 | 0 → 0 | 0 → 0 |
| mode-change | pass → pass | 12.32 → 12.31 | 0.38 → 0.37 | 11 → 11 | 0 → 0 |
| model-reset | pass → pass | 4.02 → 4.20 | 5.10 → 5.10 | 1 → 1 | — → — |
| native-reset | pass → pass | 4.03 → 4.20 | 5.10 → 5.10 | 1 → 1 | — → — |
| pitch-pulse | pass → pass | 4.26 → 4.19 | 5.62 → 5.62 | 1 → 6 | 0 → 0 |
| queued-model-reset | pass → pass | 4.39 → 4.19 | 5.10 → 5.10 | 1 → 1 | — → — |
| saturation-recovery | pass → pass | 3.77 → 3.82 | 0.17 → 0.17 | 2 → 4 | 0 → 0 |
| sensor-invalid | pass → pass | 3.90 → 3.81 | 0.12 → 0.12 | 2 → 3 | 0 → 0 |
| sensor-stale | pass → pass | 4.18 → 3.82 | 0.12 → 0.12 | 6 → 3 | 0 → 0 |
| time-reset | pass → pass | 5.04 → 4.19 | 5.08 → 5.10 | 1 → 6 | — → — |
| timing-gap | pass → pass | 3.93 → 3.81 | 0.12 → 0.12 | 6 → 3 | 0 → 0 |
| two-models | pass → pass | 3.88/3.46 → 3.83/3.82 | 0.12/0.12 → 0.12/0.12 | — → — | 0/0 → 0/0 |

- All scenarios pass on soft contact with unchanged acceptance envelopes. No envelope needed
  changing.
- Tilt, lift and stop latency are unchanged within about 1° and 0.04 m; stop latency stays 0.
- Range-limited ticks rise after drops (level-thrust, pitch-pulse, time-reset 1 → 6). The rigid
  plant stopped a fall in one 1 ms step. The soft contact spreads it over several milliseconds, so
  more samples sit above ±4 g. That matches a real airframe better. Each case still produces an
  operator notice.
- Ground chatter is gone. That removes the main source of simulated sensor noise; real airframe
  vibration must come from bench measurements (R5), not the simulator.

## Unresolved, and decisions needed

- **Default simulated sensor.** The Rust firmware configures DLPF_CFG=1 (about 184 Hz accel
  bandwidth). The legacy 0.1 filter (about 17 Hz at 1 kHz) matches the legacy *simulator*, not the
  Rust firmware's sensor configuration. Options: keep the default unfiltered, model the actual
  DLPF_CFG=1 response, or use `mpu6050-4g-lpf01` for paired comparisons only.
- **Acceptance gate.** Full-tick parity cannot pass while the dispatch-order safety correction
  exists. The existing stable-interval characterization, with no unexpected violations, is the
  defensible criterion. Changing the gate needs a recorded decision.
- **Contact chatter itself** is a Gazebo artifact. It affects every ground phase and every landing
  (see [gazebo-accel-saturation](../gazebo-accel-saturation.md)). Fixing it changes the frozen plant.
- **Hardware relevance.** A real MPU6050 always filters, and a resting airframe does not chatter at
  500 Hz. Still, the landed gate's sensitivity to vibration should be checked on the bench (R5).
- Resolved: run-to-run spread measured on x86 (10 runs each, rigid and soft), above.
