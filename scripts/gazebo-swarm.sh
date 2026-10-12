#!/bin/sh
# Gazebo with N Flix drones flown as a swarm; executed inside the Gazebo Docker image.
# The browser viewer (tools/gazebo-viewer.py --swarm) reads poses from this container and the
# coordinator's state from .local/gazebo-swarm.json.
# Usage: sh scripts/gazebo-swarm.sh [COUNT] [loop|once] [ground|onboard]
#   once: one flight, exit status = result. onboard: position loop in flight-core (swarm S2).
set -eu
count=${1:-3}
mode=${2:-loop}
loop_at=${3:-ground}
cd /work
target=/work/target/gazebo-swarm-$(uname -m)
export CARGO_TARGET_DIR="$target"
cargo build --release --locked -p flight-gazebo
cmake -S adapters/gazebo -B "$target/plugin" -DFLIX_RUST_LIBRARY="$target/release/libflight_gazebo.a"
cmake --build "$target/plugin" -j2 --target flix_rust
python3 scripts/gazebo-swarm-world.py /legacy/gazebo/models/flix /tmp/flix-swarm "$count"
export GAZEBO_MODEL_PATH=/tmp/flix-swarm/models
export GAZEBO_PLUGIN_PATH="$target/plugin"
export GAZEBO_MODEL_DATABASE_URI=''
export ALSOFT_DRIVERS=null
gzserver --verbose /tmp/flix-swarm/swarm.world &
server=$!
trap 'kill -TERM $server 2>/dev/null; exit 130' INT TERM
loop=
[ "$mode" = loop ] && loop=--loop
[ "$loop_at" = onboard ] && loop="$loop --onboard"
status=0
PYTHONPATH=adapters/crazyswarm2:tools python3 -u -m flix_swarm.gazebo --drones "$count" $loop || status=$?
# Stopping gzserver ends it by SIGTERM; that is cleanup, not the result. Report the flight.
kill -TERM $server 2>/dev/null || true
wait $server 2>/dev/null || true
exit $status
