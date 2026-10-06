#include "flix.h"
#include <arpa/inet.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <functional>
#include <cstring>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdexcept>
#include <limits>
#include <unistd.h>
namespace gazebo {
class FlixRust final : public ModelPlugin {
  physics::ModelPtr model;
  physics::LinkPtr body;
  sensors::ImuSensorPtr imu;
  event::ConnectionPtr update, time_reset;
  uint64_t handle = 0;
  int socket_fd = -1;
  int command_port = 0;
  sockaddr_in endpoint{};
  bool have_endpoint = false;
  uint64_t previous = UINT64_MAX;
  sockaddr_in trace_endpoint{};
  bool tracing = false;
  uint64_t trace_sequence = 0;
  int test_sensor_fault = 0;
  uint64_t test_fault_at = 0;
  bool test_fault_injected = false;
  uint64_t test_gap_at = 0, test_gap_us = 0;
  FlixOutput last_output{};
  bool ideal_imu = false;
  std::array<float,6> raw_imu{};
  uint32_t sensor_limit_mask = 0;
  bool open_socket() {
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) return false;
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(command_port);
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(socket_fd, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0 ||
        fcntl(socket_fd, F_SETFL, O_NONBLOCK) < 0) {
      close(socket_fd); socket_fd = -1; return false;
    }
    return true;
  }
  // Optional loopback binary tracing. No file IO or text formatting in tick.
  static void put32(uint8_t *p, uint32_t v) {
    for (int i=0;i<4;i++) p[i]=uint8_t(v>>(8*i));
  }
  static void put64(uint8_t *p, uint64_t v) {
    for (int i=0;i<8;i++) p[i]=uint8_t(v>>(8*i));
  }
  void trace_command(const uint8_t *data, int length, uint64_t now, bool accepted) {
    if (!tracing) return;
    // A rejected SETUP_SIGNING can still contain a secret. Never trace it.
    if (length >= 10 && data[0] == 253 && data[7] == 0 && data[8] == 1 && data[9] == 0) return;
    std::array<uint8_t, 308> packet{};
    std::memcpy(packet.data(), "FLXC", 4);
    put32(packet.data()+4, 1); put64(packet.data()+8, now);
    put32(packet.data()+16, accepted); put32(packet.data()+20, length);
    std::memcpy(packet.data()+24, data, length);
    sendto(socket_fd, packet.data(), 24+length, MSG_DONTWAIT,
           reinterpret_cast<sockaddr *>(&trace_endpoint), sizeof(trace_endpoint));
  }
  void trace_tick(const FlixInput &input, const FlixOutput &output) {
    if (!tracing) return;
    FlixObservation observation{};
    if (flix_observe(handle, &observation) != 0) return;
    std::array<uint8_t, 220> packet{};
    std::memcpy(packet.data(), "FLXT", 4); put32(packet.data()+4, 3);
    put64(packet.data()+8, input.now_us); put64(packet.data()+16, output.generation);
    put32(packet.data()+24, output.state); put32(packet.data()+28, output.reason);
    put32(packet.data()+32, observation.source);
    put32(packet.data()+36, observation.target_attitude_valid);
    put64(packet.data()+40, trace_sequence++);
    put64(packet.data()+48, input.sample_us);
    const auto pose = model->WorldPose();
    const auto velocity = body->WorldLinearVel();
    const float values[34] = {
      float(pose.Pos().X()), float(pose.Pos().Y()), float(pose.Pos().Z()),
      float(pose.Rot().W()), float(pose.Rot().X()), float(pose.Rot().Y()), float(pose.Rot().Z()),
      float(velocity.X()), float(velocity.Y()), float(velocity.Z()),
      input.gyro[0], input.gyro[1], input.gyro[2],
      input.acceleration[0], input.acceleration[1], input.acceleration[2],
      observation.estimate[0], observation.estimate[1], observation.estimate[2], observation.estimate[3],
      observation.rates[0], observation.rates[1], observation.rates[2],
      observation.target_rates[0], observation.target_rates[1], observation.target_rates[2],
      observation.target_attitude[0], observation.target_attitude[1], observation.target_attitude[2], observation.target_attitude[3],
      output.motors[0], output.motors[1], output.motors[2], output.motors[3]
    };
    for (int i=0;i<34;i++) {
      uint32_t bits; std::memcpy(&bits, values+i, 4); put32(packet.data()+56+4*i, bits);
    }
    for (int i=0;i<6;i++) {
      uint32_t bits; std::memcpy(&bits, &raw_imu[i], 4); put32(packet.data()+192+4*i,bits);
    }
    put32(packet.data()+216,sensor_limit_mask);
    sendto(socket_fd, packet.data(), packet.size(), MSG_DONTWAIT,
           reinterpret_cast<sockaddr *>(&trace_endpoint), sizeof(trace_endpoint));
  }
  uint32_t reported_state = UINT32_MAX, reported_reason = UINT32_MAX;

public:
  // Gazebo calls ModelPlugin::Reset for model-only and full resets. Time-only
  // resets use a separate event. Both revoke this instance's owner authority.
  void Reset() override {
    if (handle) flix_reset(handle);
    have_endpoint = false;
    previous = UINT64_MAX;
    test_fault_injected = false;
    last_output = {};
    // Owner queues are only one layer. Closing also discards unread kernel
    // datagrams, including old mode/arm packets queued while physics paused.
    if (socket_fd >= 0) { close(socket_fd); socket_fd = -1; }
    if (command_port && !open_socket())
      gzerr << "Flix command socket reset failed; control remains unavailable\n";
  }
  ~FlixRust() override {
    if (handle)
      flix_destroy(handle);
    if (socket_fd >= 0)
      close(socket_fd);
  }
  void Load(physics::ModelPtr parent, sdf::ElementPtr config) override {
    model = parent;
    body = model->GetLink("body");
    imu = std::dynamic_pointer_cast<sensors::ImuSensor>(
        sensors::get_sensor(model->GetScopedName(true) + "::body::imu"));
    if (!body || !imu)
      throw std::runtime_error("Flix body/IMU missing");
    handle = flix_create();
    if (!handle)
      throw std::runtime_error("Flix instance capacity exhausted");
    // Simulator traffic binds only loopback; each instance must use a distinct
    // port.
    int port =
        config->HasElement("udp_port") ? config->Get<int>("udp_port") : 14550;
    if (port <= 0 || port > 65535)
      throw std::runtime_error("invalid UDP port");
    command_port = port;
    if (!open_socket())
      throw std::runtime_error("UDP bind/configuration");
    int trace_port = config->HasElement("trace_port") ? config->Get<int>("trace_port") : 0;
    if (trace_port < 0 || trace_port > 65535 || trace_port == port)
      throw std::runtime_error("invalid trace port");
    tracing = trace_port != 0;
    if (config->HasElement("imu_profile")) {
      const auto profile = config->Get<std::string>("imu_profile");
      if (profile != "mpu6050-4g" && profile != "ideal")
        throw std::runtime_error("invalid IMU profile");
      ideal_imu = profile == "ideal";
    }
    // Explicit isolated test profile only; production/default SDF has no fault.
    if (config->HasElement("test_sensor_fault")) {
      const auto kind = config->Get<std::string>("test_sensor_fault");
      if (!tracing || !config->HasElement("test_fault_at_us") ||
          (kind != "stale" && kind != "invalid"))
        throw std::runtime_error("invalid instrumented sensor fault profile");
      test_sensor_fault = kind == "stale" ? 1 : 2;
      test_fault_at = config->Get<uint64_t>("test_fault_at_us");
      if (test_fault_at < 10001) throw std::runtime_error("invalid test fault time");
    }
    if (config->HasElement("test_gap_at_us")) {
      test_gap_at = config->Get<uint64_t>("test_gap_at_us");
      test_gap_us = config->Get<uint64_t>("test_gap_us");
      if (!tracing || test_gap_at < 10000 || test_gap_us < 10000 || test_gap_us > 100000)
        throw std::runtime_error("invalid instrumented owner gap profile");
    }
    trace_endpoint.sin_family = AF_INET;
    trace_endpoint.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    trace_endpoint.sin_port = htons(trace_port);
    update = event::Events::ConnectWorldUpdateBegin(
        std::bind(&FlixRust::tick, this));
    time_reset = event::Events::ConnectTimeReset([this] { Reset(); });
  }
  void tick() {
    auto now =
        static_cast<uint64_t>(model->GetWorld()->SimTime().Double() * 1e6);
    if (now == previous)
      return;
    previous = now;
    for (int i = 0; i < 4; i++) {
      std::array<uint8_t, 281> packet{};
      sockaddr_in source{};
      socklen_t size = sizeof(source);
      int length = recvfrom(socket_fd, packet.data(), packet.size(), 0,
                            reinterpret_cast<sockaddr *>(&source), &size);
      if (length <= 0)
        break;
      if (length > 280) continue;
      bool accepted = flix_receive(handle, packet.data(), length, now) == 0;
      trace_command(packet.data(), length, now, accepted);
      if (accepted) {
        endpoint = source;
        have_endpoint = true;
      }
    }
    const auto gyro = imu->AngularVelocity(), acc = imu->LinearAcceleration();
    FlixInput input{1,
                    1,
                    now,
                    now,
                    {float(gyro.X()), float(gyro.Y()), float(gyro.Z())},
                    {float(acc.X()), float(acc.Y()), float(acc.Z())}};
    sensor_limit_mask = 0;
    // Match the configured MPU6050 signed-register ranges before calibration.
    // Keep invalid nonfinite observations invalid, never turn them into bounds.
    constexpr float acc_scale = 9.80665f / 8192.0f;
    constexpr float gyro_scale = 3.14159265358979323846f / (180.0f * 16.4f);
    for (int i=0;i<3;i++) {
      raw_imu[i] = input.gyro[i]; raw_imu[i+3] = input.acceleration[i];
      if (!ideal_imu) {
        for (int kind=0;kind<2;kind++) {
          float &value = kind ? input.acceleration[i] : input.gyro[i];
          const float scale = kind ? acc_scale : gyro_scale;
          if (std::isfinite(value)) {
            const float limited = std::clamp(value,-32768.0f*scale,32767.0f*scale);
            if (limited != value) sensor_limit_mask |= 1u << (i+kind*3);
            value = limited;
          }
        }
      }
    }
    if (test_sensor_fault && !test_fault_injected && now >= test_fault_at) {
      test_fault_injected = true;
      if (test_sensor_fault == 1) input.sample_us = now - 10001;
      else input.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
    }
    FlixOutput output{};
    if (test_gap_at && now >= test_gap_at && now - test_gap_at < test_gap_us) {
      // Miss owner invocations using actual world time; never hold motor force.
      // Trace state is the last owner snapshot, with zero applied outputs.
      output = last_output;
      output.now_us = now;
      for (auto &motor : output.motors) motor = 0;
    } else {
      if (flix_tick(handle, &input, &output) != 0) return;
      last_output = output;
    }
    if (output.state != reported_state || output.reason != reported_reason) {
      gzmsg << "Flix Rust state=" << output.state << " reason=" << output.reason
            << " sim_us=" << now << " acceleration=" << acc << "\n";
      reported_state = output.state;
      reported_reason = output.reason;
    }
    constexpr double arm = .035355, max_thrust = .03 * 9.80665,
                     max_torque = .0024 * 9.80665;
    // Motor order RL, RR, FR, FL. Preserve baseline plant asymmetry; bounded
    // Rust outputs.
    constexpr double scales[4] = {.9, 1.05, 1.1, 1.0};
    constexpr double x[4] = {-arm, -arm, arm, arm},
                     y[4] = {arm, -arm, -arm, arm}, yaw[4] = {-1, 1, -1, 1};
    for (int i = 0; i < 4; i++) {
      body->AddLinkForce({0, 0, scales[i] * max_thrust * output.motors[i]},
                         {x[i], y[i], 0});
      body->AddRelativeTorque(
          {0, 0, yaw[i] * scales[i] * max_torque * output.motors[i]});
    }
    trace_tick(input, output);
    // Drain bounded replies even without a peer. Holding old telemetry here
    // fills the slots and drops the first command ACK on a late connection.
    for (int i = 0; i < 4; i++) {
      std::array<uint8_t, 280> packet{};
      int length = flix_reply(handle, packet.data(), packet.size());
      if (length <= 0)
        break;
      if (have_endpoint)
        sendto(socket_fd, packet.data(), length, MSG_DONTWAIT,
               reinterpret_cast<sockaddr *>(&endpoint), sizeof(endpoint));
    }
  }
};
GZ_REGISTER_MODEL_PLUGIN(FlixRust)
} // namespace gazebo
