.PHONY: check demo build sim-check sim-scenarios swarm-sim
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

# Paired C++/Rust comparison (fixtures/simulator/acceptance.json comparison). Both
# engines use the default soft-contact plant; rigid contact made the gate flaky.
PARITY_SCENARIOS := pitch-pulse level-thrust
sim-parity:
	docker build -t flix-rust-gazebo:local -f adapters/gazebo/Dockerfile .
	# Prepare inside the container: earlier container steps leave .local owned by root.
	docker run --rm -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh -c \
	  'git config --global --add safe.directory /legacy && mkdir -p .local && git -C /legacy archive --format=tar 1f5b268271536e45a9377d1c5317d3eb6f3c3bba > .local/legacy-baseline.tar && python3 scripts/fetch-legacy-mavlink.py > /dev/null'
	set -e; status=0; for s in $(PARITY_SCENARIOS); do \
	  docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-legacy-scenario.sh $$s /work/.local/qualification/parity/legacy-$$s || true; \
	  docker run --rm --init -v "$(CURDIR):/work" -v "$(CURDIR)/../flix:/legacy:ro" -w /work flix-rust-gazebo:local sh scripts/gazebo-scenario.sh $$s /work/.local/qualification/parity/rust-$$s; \
	  docker run --rm -v "$(CURDIR):/work" -w /work flix-rust-gazebo:local python3 tools/gazebo-compare.py .local/qualification/parity/rust-$$s/trace.jsonl .local/qualification/parity/legacy-$$s/trace.jsonl --output .local/qualification/parity/compare-$$s.json > /dev/null || status=1; \
	  python3 -c "import json,sys;r=json.load(open(sys.argv[1]));s=r['stable_interval_characterization'];d=r['dispatch_difference_classification'];print('parity',sys.argv[2],r['acceptance']['status'],'stable',{k:round(v,4) for k,v in s['metrics'].items()},'unexpected',len(d['unexpected_violation_ticks']),d['unexpected_violation_ticks'][:12],'unmatched',r['boundary_unmatched_ticks'][:12])" .local/qualification/parity/compare-$$s.json $$s; \
	done; exit $$status

SWARM_SCENARIOS := formation stop partial-arm
# Crazyswarm2 coordinating simulated Flix drones (swarm plan S1). Results: .local/qualification/swarm-s1/.
swarm-sim:
	docker build -t flix-crazyswarm2:local -f adapters/crazyswarm2/Dockerfile adapters/crazyswarm2
	set -e; for s in $(SWARM_SCENARIOS); do \
	  docker run --rm --init -v "$(CURDIR):/work" -w /work flix-crazyswarm2:local sh scripts/crazyswarm2-scenario.sh $$s /work/.local/qualification/swarm-s1/crazyswarm2-$$s; \
	done
