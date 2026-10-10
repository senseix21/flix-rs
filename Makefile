.PHONY: check demo build sim-check sim-scenarios
check:
	sh scripts/check.sh
demo:
	cargo run --release -p flight-replay --bin flight-replay -- --demo
build:
	cargo build --workspace --release --locked

sim-check:
	docker build -t flix-rust-gazebo:local -f adapters/gazebo/Dockerfile .
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-smoke.sh

sim-scenarios:
	docker build -t flix-rust-gazebo:local -f adapters/gazebo/Dockerfile .
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh level-thrust /work/.local/qualification/level-thrust
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh pitch-pulse /work/.local/qualification/pitch-pulse
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh level-thrust /work/.local/qualification/native-reset native

	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh acro-level /work/.local/qualification/acro-level

	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh auto-level /work/.local/qualification/auto-level

	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh lease-loss-reconnect /work/.local/qualification/lease-loss-reconnect
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh level-thrust /work/.local/qualification/time-reset time
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh level-thrust /work/.local/qualification/model-reset model
	docker run --rm --init -e FLIX_TWO_MODELS=1 -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh two-models /work/.local/qualification/two-models
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh sensor-invalid /work/.local/qualification/sensor-invalid
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh sensor-stale /work/.local/qualification/sensor-stale
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh level-thrust /work/.local/qualification/queued-model-reset queued-model
	docker run --rm --init -e FLIX_LATE_CLIENT=1 -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh late-client /work/.local/qualification/late-client
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh saturation-recovery /work/.local/qualification/saturation-recovery
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh mode-change /work/.local/qualification/mode-change
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh timing-gap /work/.local/qualification/timing-gap
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh diagnostic-stop /work/.local/qualification/diagnostic-stop
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh landing-contact /work/.local/qualification/landing-contact
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh long-level /work/.local/qualification/long-level
	docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh axis-pulses /work/.local/qualification/axis-pulses

# Paired C++/Rust comparison (fixtures/simulator/acceptance.json comparison). The Rust
# side uses the legacy-equivalent accelerometer profile so only controllers differ.
PARITY_SCENARIOS := pitch-pulse level-thrust
sim-parity:
	docker build -t flix-rust-gazebo:local -f adapters/gazebo/Dockerfile .
	mkdir -p .local
	git -C ../flix archive --format=tar 1f5b268271536e45a9377d1c5317d3eb6f3c3bba > .local/legacy-baseline.tar
	python3 scripts/fetch-legacy-mavlink.py
	set -e; for s in $(PARITY_SCENARIOS); do \
	  docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-legacy-scenario.sh $$s /work/.local/qualification/parity/legacy-$$s || true; \
	  docker run --rm --init -e FLIX_IMU_PROFILE=mpu6050-4g-lpf01 -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh $$s /work/.local/qualification/parity/rust-$$s; \
	  python3 tools/gazebo-compare.py .local/qualification/parity/rust-$$s/trace.jsonl .local/qualification/parity/legacy-$$s/trace.jsonl --output .local/qualification/parity/compare-$$s.json > /dev/null; \
	  echo "parity $$s: acceptance pass"; \
	done
