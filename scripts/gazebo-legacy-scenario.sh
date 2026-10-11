#!/bin/sh
set -eu
cd /work
mkdir -p /tmp/flix-legacy
 tar -xf .local/legacy-baseline.tar -C /tmp/flix-legacy
python3 scripts/instrument-legacy.py /tmp/flix-legacy
python3 scripts/gazebo-contact.py /tmp/flix-legacy/gazebo/models/flix/flix.sdf "${FLIX_CONTACT:-soft}"
cmake -S /tmp/flix-legacy/gazebo -B /tmp/flix-legacy-build -DCMAKE_CXX_FLAGS='-I/work/.local/legacy-mavlink/MAVLink'
cmake --build /tmp/flix-legacy-build -j2
export GAZEBO_MODEL_PATH=/tmp/flix-legacy/gazebo/models
export GAZEBO_PLUGIN_PATH=/tmp/flix-legacy-build
export GAZEBO_MODEL_DATABASE_URI=''
export ALSOFT_DRIVERS=null
mkdir -p "$2"
gzserver --seed 24301 --verbose adapters/gazebo/bench.world > "$2/gzserver.log" 2>&1 &
gz_pid=$!
trap 'kill "$gz_pid" 2>/dev/null || true' EXIT
PYTHONPATH=tools python3 tools/gazebo-scenario.py --engine legacy --scenario "$1" --output "$2"
