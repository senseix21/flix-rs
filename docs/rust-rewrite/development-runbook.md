# Development build and recovery

This is an unqualified development workflow. It does not authorize physical
flight. Read the [execution report](execution-report.md) and
[qualification procedure](qualification.md) before connecting motor power.

## Host and simulator

```sh
sh scripts/check.sh
cargo check --manifest-path apps/esp32/firmware/Cargo.toml --locked --target thumbv7em-none-eabihf
make sim-check
make sim-scenarios
```

The simulator commands use isolated containers without published browser ports.
Keep the existing browser session separate. `sim-scenarios` fails when an
acceptance threshold or trace completeness check fails; preserve the failed
directory before rerunning it. Run qualification traces without competing
compilation workloads. A missing trace is missing evidence, even when the observed
motors stopped.

## Board preparation

Complete `fixtures/hardware/esp32s3-mpu6050.json` from the delivered board,
schematic and measurements. Check it with:

```sh
python3 scripts/hardware-profile.py fixtures/hardware/esp32s3-mpu6050.json
```

The shipped template deliberately fails. A successful validation establishes a
complete input record, not electrical or flight qualification. Review its pins
against the board schematic and the SDK configuration. The default SDK profile
has all pins unwired and PWM disabled. Example compile profiles are not wiring
instructions.

Use the pinned ESP-IDF v5.4.2 environment, Rust `esp` 1.88.0.0, and the ESP clang
library path. Activate the matching IDF Python environment before exporting IDF
tools; a different Python minor version selects a different environment.

```sh
sh scripts/build-esp32.sh
```

This builds and links; it does not flash. Inspect flash size, partitions, GPIOs,
gate polarity and motor-power isolation before a separate physical upload.

## Radio enrollment

Follow [ADR 002](adr-002-wired-session.md). Enrollment uses the direct trusted
UART0 interface while stopped in maintenance. A proxy cannot enroll the board.
Generate a fresh private session file on every boot; never reuse an old key.

```sh
PYTHONPATH=tools python3 tools/enroll-radio.py --device /dev/cu.YOUR_UART --session-file .local/flight.session.json
PYTHONPATH=tools python3 tools/enroll-radio.py --device /dev/cu.YOUR_UART --revoke
```

The private JSON contains the key for a signed `Flix` client. Load it locally
with `json.load`, convert `key` using `bytes.fromhex`, and pass it as `key=` to
`Flix` with the chosen UDP or explicitly marked proxy transport. Do not put keys
in shell arguments, repository files, screenshots or evidence traces. The
client validates incoming signatures, source and replay before completing a
request. Actual QGroundControl compatibility remains unverified.

If writing the session file fails after enrollment, the tool attempts revocation
while still in maintenance. A lost cable can prevent that request. Keep motors
stopped and reboot the board to restore unprovisioned radio, then enroll a fresh
session. An interruption is not proof that revocation reached the board.

## Instrumentation and battery

The battery guard is opt-in through the Rust owner in stopped maintenance. No ADC
pin, divider, calibrated acquisition or airframe threshold is installed by
default. Installing it requires measured voltage scaling and an accepted loss
policy. Unknown, invalid or stale voltage denies arming when the guard is enabled.
Critical voltage latches the common power fault; recovery does not arm.

`flix_board_metrics` exports 52 integer values only from stopped maintenance on
the owner task: version, five histograms (count, maximum, eight buckets each),
then invalid sample-age count. Histograms cover acquisition, sample age, owner
tick duration, total acquisition/owner duration, and schedule jitter. The field
named `core` measures the whole owner tick including dispatch and telemetry; it
is not a pure control-core benchmark. Counters are bounded and saturating. This
export is an instrumentation boundary, not a physical stop measurement.

The wired capture application now requests a frozen snapshot in exclusive
maintenance and checks its nonce, eight chunks and histogram consistency:

```sh
PYTHONPATH=tools python3 tools/capture-metrics.py --device /dev/cu.YOUR_UART --output .local/timing-snapshot.json
```

It disarms, enters maintenance, captures, and exits. The output path must be new.
A failed operation can leave the board stopped in maintenance. Snapshot counters
are cumulative; take before/after captures to describe a particular workload.
This command does not run the 30-minute workload, measure physical stop, or
establish stack/heap high-water marks. The actual UART capture remains unverified
until performed on the delivered board.

## Bundle and rollback

```sh
python3 scripts/development-bundle.py --artifact apps/esp32/idf/build/flix_rust.bin --output .local/development-bundle
```

Use a new output directory for each build. Keep the complete IDF build directory
(bootloader, partition table, application, flasher arguments, `sdkconfig`) with
its source/configuration manifest. The bundle script packages the named artifact
and contracts; it is not a complete flash or qualified release package.

Before physical installation, save the currently verified image, its partitions
and exported settings. Rebuild in a fresh checkout from pinned dependencies,
compare manifests, and perform stopped boot/IMU checks with motor power isolated.
Use the matching IDF flasher arguments for the reviewed board. Actual installation
and rollback must be recorded on that board; neither has been performed here.

## Optional paired proxy

See [proxy build and pairing instructions](../../apps/proxy/README.md) and
[verification scope](proxy-integration.md). Build the separate proxy image with
`scripts/build-proxy.sh`; use `FLIX_ESPNOW_ENABLE` for the flight receiver profile.
The default profile keeps it disabled. Saved transport provisioning executes only
in stopped Maintenance and must be measured on the actual board. If it triggers
a timing fault, retain the fault trace and use explicit recovery; do not relax
the gap limit to conceal provisioning latency.

## Complete local software package

Use `python3 scripts/software-package.py create NEW_PACKAGE --build proxy=PROXY_BUILD --build flight-default=FLIGHT_BUILD --build flight-espnow=RADIO_BUILD`.
Each SDK build directory must contain its reviewed `sdkconfig`, complete
`flasher_args.json` file set, and `flash_args`. Copy the SDK config into the build
directory when ESP-IDF stores it elsewhere. The command excludes ignored/private
provisioning files and refuses network passwords in SDK config.

Run `verify PACKAGE`, then `extract PACKAGE NEW_SOURCE_DIRECTORY` to verify and
extract its source archive. Build from that directory using `sh scripts/check.sh`;
set `FLIX_LEGACY` to the inspected pinned C++ repository for the scalar oracle.
SDK builds require the toolchains documented above; a clean host build does not
establish a byte-identical firmware rebuild.

`install PACKAGE DESTINATION` stages and verifies before replacing a local package.
The previous verified package is preserved as `DESTINATION.previous`.
`rollback DESTINATION` verifies and swaps those packages. Both are local operations;
no board is flashed. A crash during the rename sequence can leave `.swap`;
preserve all copies and verify them before manual recovery. Private peer NVS and
fresh per-boot MAVLink enrollment remain separate operator-owned provisioning.
