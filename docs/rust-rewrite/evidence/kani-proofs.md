# Kani model-checking proofs

Date: 2026-10-10. Tool: Kani 0.68.0 (CBMC 6.11.0), macOS arm64. Harnesses:
`crates/flight-core/src/proofs.rs`, compiled only under `cfg(kani)`. Run with
`cd crates/flight-core && cargo kani -Z stubbing` (stubbing since 2026-10-13). CI job `kani` runs the same command.
Requirements: SAF-01, SAF-02, SAF-05, SAF-08. Hardware profile: none.

## What a proof adds

The property tests sample random operation sequences. A Kani proof checks a harness for every
value of its symbolic inputs: every `f32` bit pattern including NaN, ±infinity and subnormals,
every `u64` time, every lifecycle state.

| Harness | Property | Time |
| --- | --- | ---: |
| `mix_outputs_are_bounded` | For any thrust and torque, `mix` returns `None` or four finite outputs in [0, 1] | 0.8 s |
| `clock32_extension_is_monotonic` | `Clock32::extend` never panics; a forward step ≤ 2³¹−1 advances by exactly that step; a larger (backward) step is refused | 0.1 s |
| `commit_is_bounded_and_stopped_states_are_zero` | For any frame, state, lease, sample, diagnostic and time, `commit` returns finite values in [0, 1], and exactly zero outside Armed/Maintenance | 0.6 s |
| `commit_rejects_foreign_or_stale_authority` | A frame from another issuer, an older generation or an issuer-0 owner always commits to zero | 0.5 s |
| `only_arm_enters_armed_and_disarm_always_leaves` | From any state, `disarm` leaves Armed and Maintenance; `fault`, `clear_fault`, `select`, `maintenance`, `reset` and `reconfigure` never enter Armed | 1.1 s |
| `position_limit_bounds_thrust_and_refuses_non_finite` (swarm S2, 2026-10-13) | For any demand, the position limiter refuses non-finite input or returns a finite force with vertical component at least 0.2 g and thrust in [0, `max_thrust`]. `libm::sqrtf` and `tanf` are replaced by contract stubs (IEEE NaN/infinity behaviour; finite results within 2^-20 relative); run with `cargo kani -Z stubbing`. The tilt bound timed out after 20 minutes and is checked by the property test `limiter_bounds_tilt_and_thrust` instead | 2.2 s |

The five original harnesses pass; with the later additions, all 8 harnesses passed on 2026-10-13.

## Scope

- `any_flight()` makes lifecycle, issuer, generation, source, lease, sample, health, calibration
  and diagnostic state symbolic. Estimator and controller float state stay at their defaults,
  which the proved properties do not read. `tick` and `arm` are not covered: they run the
  estimator and controller, which is float-heavy and better served by the property tests.
- The lease intent is symbolic only in thrust. `commit` reads only whether a lease is fresh.
- To let the in-crate harness construct state, the fields of `Flight`, `MotorFrame` and `Lease`
  became `pub(crate)`. They remain private outside `flight-core`, so other crates still cannot
  forge a `MotorFrame`.

## Checking the proofs can fail

Each mutation was applied, the relevant harness run, and the source restored.

| Planted bug | Harness | Result |
| --- | --- | --- |
| Mixer stops clamping | mix | Caught |
| Clock accepts backward steps | clock32 | Caught |
| `commit` drops its range check | commit bounded | Caught |
| `commit` drops its state check | commit bounded | Caught |
| `commit` drops its generation check | commit authority | Caught |
| `commit` drops its issuer check | commit authority | Caught |
| `disarm` leaves Armed unchanged | lifecycle | Caught |
| `clear_fault` enters Armed | lifecycle | Caught |

The property-test mutation run could not detect the range-check and state-check removals, because
frames produced by `tick` are already bounded and stopped. Kani catches both because it checks
`commit` against every possible frame. Together, the two methods show that each layer of the
gate holds on its own, not just that the layers back each other up.

## Unresolved

- No proof covers `tick`, `arm`, `command` or the estimator and controller numerics.
- Kani's first run used unwind 5, too small for `Tuning::valid`'s six-vector loop (an unwinding
  assertion, not a property failure). The lifecycle harness now uses unwind 7.
