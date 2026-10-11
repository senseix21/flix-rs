#!/bin/sh
# Persistent development simulation; executed inside the Gazebo Docker image.
set -eu
cd /work
export CARGO_TARGET_DIR=/work/target/gazebo-linux
cargo build --release --locked -p flight-gazebo
cmake -S adapters/gazebo -B /work/target/gazebo-plugin -DFLIX_RUST_LIBRARY="$CARGO_TARGET_DIR/release/libflight_gazebo.a"
cmake --build /work/target/gazebo-plugin -j2
mkdir -p /tmp/flix-models
cp -R /legacy/gazebo/models/flix /tmp/flix-models/
sed -i 's/libflix.so/libflix_rust.so/' /tmp/flix-models/flix/flix.sdf
python3 scripts/gazebo-contact.py /tmp/flix-models/flix/flix.sdf "${FLIX_CONTACT:-soft}"
export GAZEBO_MODEL_PATH=/tmp/flix-models
export GAZEBO_PLUGIN_PATH=/work/target/gazebo-plugin
export GAZEBO_MODEL_DATABASE_URI=''
export ALSOFT_DRIVERS=null
exec python3 tools/gazebo-server.py
