#!/bin/sh
# Run one Crazyswarm2 S1 scenario inside adapters/crazyswarm2/Dockerfile.
#   sh scripts/crazyswarm2-scenario.sh formation|stop|partial-arm OUTPUT_DIR [ground|drone]
# The third argument chooses where the position loop runs (default ground, swarm S1).
set -eu
scenario=$1
out=$2
export FLIX_POSITION_LOOP=${3:-ground}
cd /work
export CARGO_TARGET_DIR=/work/target/crazyswarm2-linux
cargo build --release --locked -p flight-gazebo
export FLIX_LIBRARY="$CARGO_TARGET_DIR/release/libflight_gazebo.so"
set +u
. /opt/ros/jazzy/setup.sh
. /opt/ws/install/setup.sh
set -u
export PYTHONPATH="/work/adapters/crazyswarm2:/opt/crazyflie-firmware/build:${PYTHONPATH:-}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=42
rm -rf "$out"
mkdir -p "$out"
export FLIX_SWARM_TRACE="$out"
ros2 launch adapters/crazyswarm2/ros/flix_swarm.launch.py > "$out/server.log" 2>&1 &
server=$!
status=0
timeout 300 python3 adapters/crazyswarm2/ros/demo.py --scenario "$scenario" \
  --ros-args -p use_sim_time:=true > "$out/demo.log" 2>&1 || status=$?
# A non-interactive shell starts background jobs with SIGINT ignored; use SIGTERM.
kill -TERM "$server" 2>/dev/null || true
pkill -TERM -f crazyflie_server 2>/dev/null || true
wait "$server" 2>/dev/null || true
if [ "$status" -ne 0 ]; then
  echo "coordinator failed ($status)"; tail -20 "$out/demo.log"; tail -20 "$out/server.log"
  exit "$status"
fi
python3 adapters/crazyswarm2/ros/check.py --scenario "$scenario" --trace "$out"
