# R1 simulator experiments — 2026-10-05

Status: development evidence. No hardware, radio, electrical stop or sustained
flight qualification is claimed. Sources remain uncommitted.

## Contract and provenance

The [acceptance fixture](../../../fixtures/simulator/acceptance.json) fixes inputs,
duration, units and exploratory thresholds. The instrumented profile is Gazebo 11,
the legacy 65 g plant, ground start, 1 ms ODE step, 20 ms commands and 1 ms trace.
Frames are FLU; quaternions are wxyz; motors are RL/RR/FR/FL.

[Source hashes and host toolchain](physics-r1/source-inputs.json),
[artifact hashes](physics-r1/sha256.json), [host check](physics-r1/host-check.txt)
and [physics build/run log](physics-r1/physics-check.txt) are retained. The source
manifest fingerprints uncommitted files rather than inventing a revision.
It describes the host replay artifact, not an ESP32 flight release.

The paired C++ baseline is revision
`1f5b268271536e45a9377d1c5317d3eb6f3c3bba`. A disposable archive copy was instrumented;
the user's legacy worktree was not edited. MAVLink 2.0.33 came from the Arduino
library index with its archive checksum verified; see
[library provenance](physics-r1/legacy-library.json).

The Rust plugin uses an opt-in, nonblocking loopback binary trace. Collection and
file writes occur outside the flight tick. This adds observation/UDP overhead;
it is not hardware timing evidence. Command receipts retain simulator admission
time; legacy receipts describe parsed packets, not Rust authorization semantics.

## Results

| Experiment | Result | Observation |
| --- | --- | --- |
| Rust level thrust | Pass | 7,504 ticks; maximum commanded tilt 4.494°; 0 µs observed stop latency |
| Rust pitch pulse | Pass | 7,154 ticks; maximum commanded tilt 4.246°; 0 µs observed stop latency |
| Rust native world reset | Authority/pose contract passed | Reset cleared source and outputs; final state was a latched sensor fault |
| Legacy level/pitch probes | Fail Rust stopped-state contract | Nonzero motors persisted for one tick after legacy disarm; 1,000 µs stop latency |
| Paired level thrust | Full comparison failed | Maximum motor difference 0.4630; attitude difference 0.2268°; position difference 0.1097 m |
| Paired pitch pulse | Full comparison failed | Maximum motor difference 0.5004; attitude difference 0.2337°; position difference 0.1487 m |

There were zero dropped tick records and zero invalid packets in the three Rust
runs. Their commanded intervals did not invert or fault. Ground impact after
explicit Stop produced sensor faults in the ordinary probes with motors remaining
zero. The original browser flip was not reproduced: its command sequence was not
recorded, so these candidate probes cannot establish its cause.

The paired limits were 5° attitude, 0.1 absolute motor difference and 0.5 m position.
The earliest failures coincide with changed commands and the legacy dispatch
order (MAVLink after control). Excluding explicitly identified transition ticks
only for characterization yields motor differences 0.0692 and 0.0604. The full
comparison remains failed. The legacy acceleration filter remains active; the
Rust adapter receives raw acceleration. Hidden legacy truth-yaw correction and
legacy disk logging were disabled in this comparison copy. These differences
prevent claiming identical controller inputs or complete plant parity.

The bounded `flix-world-reset` helper completed in the isolated container. After
the time regression, pose was upright, source absent and motors zero. A virtual
IMU acceleration spike subsequently latched fault reason 5. The final state needs
explicit operator recovery; automatic fault clearing was not added. The browser
container restart fallback remains in place. Time-only/model-only resets and
multiple actual models are still open.

## Reproduction and raw evidence

```sh
sh scripts/check.sh
make sim-scenarios
git -C ../flix archive 1f5b268271536e45a9377d1c5317d3eb6f3c3bba > .local/legacy-baseline.tar
docker run --rm -v "$PWD:/work" -w /work flix-rust-gazebo:local python3 scripts/fetch-legacy-mavlink.py
docker build -t flix-rust-gazebo-legacy:local -f adapters/gazebo/Legacy.Dockerfile .
docker run --rm --init -v "$PWD:/work" -w /work flix-rust-gazebo-legacy:local sh scripts/gazebo-legacy-scenario.sh level-thrust .local/qualification/legacy-level-thrust
docker run --rm --init -v "$PWD:/work" -w /work flix-rust-gazebo-legacy:local sh scripts/gazebo-legacy-scenario.sh pitch-pulse .local/qualification/legacy-pitch-pulse
python3 tools/gazebo-compare.py .local/qualification/level-thrust/trace.jsonl .local/qualification/legacy-level-thrust/trace.jsonl --output .local/qualification/level-comparison.json
python3 tools/gazebo-compare.py .local/qualification/pitch-pulse/trace.jsonl .local/qualification/legacy-pitch-pulse/trace.jsonl --output .local/qualification/pitch-comparison.json
```

The legacy scenario and comparison commands intentionally exit 1 for the failures
above. They are not successful qualification gates. Builds expose an upstream
legacy metadata warning about an overread; that command was not exercised here.
No remote CI run was performed; the local workflow now includes Rust scenarios.

The host check passed 35 Rust tests, 19 Python tests, 400 scalar oracle cases and
14 ABI simulator scenarios. Actual Gazebo runs are separate from those ABI tests.
No control gains, motor order or board pins were changed for these experiments.

Raw JSONL is gzip compressed without removing ticks or command events:

* [Rust level](physics-r1/level-thrust/trace.jsonl.gz), [result](physics-r1/level-thrust/result.json)
* [Rust pitch](physics-r1/pitch-pulse/trace.jsonl.gz), [result](physics-r1/pitch-pulse/result.json)
* [Native reset](physics-r1/native-reset/trace.jsonl.gz), [result](physics-r1/native-reset/result.json)
* [Legacy level](physics-r1/legacy-level-thrust/trace.jsonl.gz), [result](physics-r1/legacy-level-thrust/result.json)
* [Legacy pitch](physics-r1/legacy-pitch-pulse/trace.jsonl.gz), [result](physics-r1/legacy-pitch-pulse/result.json)
* [Level comparison](physics-r1/level-comparison.json), [pitch comparison](physics-r1/pitch-comparison.json)

R1 remains open for the original flip, full comparison, broader modes/commands,
loss/overload tests, additional reset forms and model isolation. R2's
[provisioning audit](../radio-provisioning-audit.md) identifies the next host work.
