#!/bin/sh
# Isolated Docker run; never target the browser's active container.
set -eu
cd /work
export CARGO_TARGET_DIR=/work/target/gazebo-linux
cargo build --release --locked -p flight-gazebo
cmake -S adapters/gazebo -B /work/target/gazebo-plugin -DFLIX_RUST_LIBRARY="$CARGO_TARGET_DIR/release/libflight_gazebo.a"
cmake --build /work/target/gazebo-plugin -j2
mkdir -p "$2"
python3 scripts/release-manifest.py /work/target/gazebo-linux/release/libflight_gazebo.a --profile "scenario-$1" > "$2/source-manifest.json"
mkdir -p /tmp/flix-models
cp -R /legacy/gazebo/models/flix /tmp/flix-models/
python3 - "$1" <<'PY'
import xml.etree.ElementTree as ET
p='/tmp/flix-models/flix/flix.sdf'
tree=ET.parse(p);plugin=tree.find(".//plugin[@name='flix']")
assert plugin is not None
plugin.set('filename','libflix_rust.so')
ET.SubElement(plugin,'trace_port').text='14560'
import os
ET.SubElement(plugin,'imu_profile').text=os.environ.get('FLIX_IMU_PROFILE','mpu6050-4g')
import json,sys
scenario=next((s for s in json.load(open('/work/fixtures/simulator/acceptance.json'))['scenarios'] if s['name']==sys.argv[1]),None)
if scenario and 'sensor_fault' in scenario:
    ET.SubElement(plugin,'test_sensor_fault').text=scenario['sensor_fault']
    ET.SubElement(plugin,'test_fault_at_us').text=str(scenario['sensor_fault_at_us'])
if scenario and 'timing_gap_at_us' in scenario:
    ET.SubElement(plugin,'test_gap_at_us').text=str(scenario['timing_gap_at_us'])
    ET.SubElement(plugin,'test_gap_us').text=str(scenario['timing_gap_us'])
tree.write(p)
import os, shutil
if os.environ.get('FLIX_TWO_MODELS')=='1':
    shutil.copytree('/tmp/flix-models/flix','/tmp/flix-models/flix-b')
    p='/tmp/flix-models/flix-b/flix.sdf';tree=ET.parse(p)
    tree.find('model').set('name','flix-b')
    plugin=tree.find(".//plugin[@name='flix']")
    port=plugin.find('udp_port')
    if port is None:port=ET.SubElement(plugin,'udp_port')
    port.text='14552';plugin.find('trace_port').text='14562';tree.write(p)
    world=ET.parse('/work/adapters/gazebo/bench.world')
    # Keep the 1 ms physics step, pace this two-model trace workload at 0.5x wall time.
    world.find('.//real_time_update_rate').text='500'
    include=ET.SubElement(world.find('world'),'include')
    ET.SubElement(include,'uri').text='model://flix-b'
    ET.SubElement(include,'pose').text='0 2 0.0138 0 0 0'
    world.write('/tmp/flix-two-models.world')
PY
export GAZEBO_MODEL_PATH=/tmp/flix-models
export GAZEBO_PLUGIN_PATH=/work/target/gazebo-plugin
export GAZEBO_MODEL_DATABASE_URI=''
export ALSOFT_DRIVERS=null
mkdir -p "$2"
world=adapters/gazebo/bench.world
test "${FLIX_TWO_MODELS:-0}" != 1 || world=/tmp/flix-two-models.world
gzserver --seed 24301 --verbose "$world" > "$2/gzserver.log" 2>&1 &
gz_pid=$!
trap 'kill "$gz_pid" 2>/dev/null || true' EXIT
if test "${FLIX_TWO_MODELS:-0}" = 1; then
    PYTHONPATH=tools python3 tools/gazebo-isolation.py --output "$2"
elif test "${FLIX_LATE_CLIENT:-0}" = 1; then
    PYTHONPATH=tools python3 tools/gazebo-late-client.py --output "$2"
elif test "$#" -gt 2; then
    kind=${3}
    test "$kind" != native || kind=all
    queued=
    if test "$kind" = queued-model; then kind=model; queued=--queued-reset; fi
    PYTHONPATH=tools python3 tools/gazebo-scenario.py --native-reset --recover-reset $queued --reset-kind "$kind" --scenario "$1" --output "$2"
else
    PYTHONPATH=tools python3 tools/gazebo-scenario.py --scenario "$1" --output "$2"
fi
