# Engineering Instructions for Flix Rust

## Mission

Build a small, understandable Rust flight application for the Flix ESP32 quadcopter. Deliver runnable slices with explicit ownership, bounded execution, reliable disarming, and measured hardware behavior.

Apply the engineering habits associated with Jon Gjengset's teaching: understand the machinery beneath abstractions, make costs and lifetimes visible, encode meaningful invariants in types, and distinguish assumptions from evidence. These instructions describe this project's approach, not Jon's personal design decisions or Helsing's internal architecture.

## Sources of truth

Follow the user's current instructions first, then accepted requirements and decisions, then this file and existing implementation. Read `docs/rust-rewrite-spec.md` and the relevant phase in `docs/rust-rewrite-plan.md` before implementation. `docs/architecture-review.md` describes the legacy C++ repository; `docs/legacy-source.md` identifies its exact revision.

The specification and plan are proposed. Record confirmed decisions and measured evidence as work progresses. Preserve user edits. Continue authorized phase work through its locally implementable gate; document unavailable hardware evidence and continue independent host work.

## Repository structure

The Cargo workspace will live at the repository root. Planned packages are `crates/flight-core`, `crates/flight-protocol`, `apps/replay`, `apps/esp32`, and `adapters/gazebo`. Create each package with its first real executable slice rather than adding empty scaffolding. Keep host test commands independent of board-specific compiler flags.

## Architecture and safety

* One owner controls flight state; communication produces validated inputs rather than mutating arming or motors.
* Every motor output, including diagnostics, passes through one final safety policy. Disarm must dominate intent selection, invalid state, and stale output frames.
* Use explicit control intents and lifecycle transitions. NaN must not select a controller path.
* Validate numerical values, source authority, field bounds, configuration, and freshness before state mutation.
* Use integer monotonic time and explicitly handle stale samples, clock discontinuities, and command leases. Heartbeats do not renew control authority.
* Keep the portable core `no_std` and free of heap allocation in the flight tick. Bound every queue and define overflow behavior.
* Keep networking, formatting, flash writes, calibration, and large transfers out of the armed control path.
* Preserve FLU coordinates and rear-left/rear-right/front-right/front-left motor order across firmware, simulation, and protocol boundaries.
* Keep unsafe code at audited hardware/FFI boundaries with a documented safety argument. Types and `Drop` do not replace physical output-stop verification.
* Separate nominal behavior parity from intentional safety corrections. Rust compilation and simulation do not establish flight qualification.

## Workflow and verification

Reproduce, define a narrow contract, test it, implement the smallest coherent behavior, integrate, and measure. Use traits at real hardware or test boundaries. Avoid speculative frameworks and unrelated tuning changes.

Once the packages exist, run the applicable formatting, Clippy, host tests, `no_std` checks, replay comparisons, and exact target builds described in the plan. Do not combine incompatible chip or host/embedded features with a blanket all-features command. Tests must cover the safety and failure contract rather than mirror implementation details.

Record gate evidence under `docs/rust-rewrite/evidence/` when actual evidence exists. Include revision, requirement IDs, hardware profile, exact commands, workload, raw traces, thresholds, results, and unresolved items. Report factual verification and leave hardware-dependent gates open until measured.
