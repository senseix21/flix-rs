# Crazyswarm2 simulator with Flix drones (swarm plan S1-08). Build context: adapters/crazyswarm2.
# Multi-arch base: native arm64 under Colima on Apple Silicon, amd64 in CI.
FROM ros:jazzy-ros-base-noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      git build-essential swig python3-dev python3-setuptools python3-numpy python3-pip \
      python3-transforms3d python3-yaml curl ca-certificates \
 && rm -rf /var/lib/apt/lists/*
RUN pip3 install --break-system-packages --no-cache-dir rowan==1.3.2
# Crazyswarm2's documented firmware release for the simulator bindings.
ARG FIRMWARE=2025.02
RUN git clone --branch $FIRMWARE --depth 1 --recursive --shallow-submodules \
      https://github.com/bitcraze/crazyflie-firmware.git /opt/crazyflie-firmware \
 && cd /opt/crazyflie-firmware && make cf2_defconfig && make bindings_python
ARG CRAZYSWARM2=f1e09954f56cd08e7d9aff5e13dc9a68795914a3
RUN git clone https://github.com/IMRCLab/crazyswarm2.git /opt/ws/src/crazyswarm2 \
 && git -C /opt/ws/src/crazyswarm2 checkout --quiet $CRAZYSWARM2
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup.sh \
 && sh /tmp/rustup.sh -y --default-toolchain 1.92.0 --profile minimal
ENV PATH=/root/.cargo/bin:$PATH
COPY ros/ /opt/flix-ros/
RUN cd /opt/ws/src/crazyswarm2 \
 && git apply /opt/flix-ros/crazyflie_server.patch \
 && cp /opt/flix-ros/flix_sil.py crazyflie_sim/crazyflie_sim/flix_sil.py \
 && cp /opt/flix-ros/flix_backend.py crazyflie_sim/crazyflie_sim/backend/flix.py \
 && . /opt/ros/jazzy/setup.sh && cd /opt/ws \
 && colcon build --packages-select crazyflie_interfaces crazyflie_py crazyflie_sim
WORKDIR /work
