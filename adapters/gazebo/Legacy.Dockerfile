FROM flix-rust-gazebo:local
RUN apt-get update && apt-get install -y --no-install-recommends libsdl2-dev && rm -rf /var/lib/apt/lists/*
