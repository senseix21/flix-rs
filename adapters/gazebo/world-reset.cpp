// Bounded external test client. It does not run in the flight tick.
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <chrono>
#include <thread>
#include <string>
#include <atomic>
namespace {
std::atomic<bool> paused_observed{false};
void on_stats(ConstWorldStatisticsPtr &message) {
  if (message->paused()) paused_observed.store(true);
}
}
int main(int argc, char **argv) {
  const std::string kind = argc > 1 ? argv[1] : "all";
  if (kind != "all" && kind != "time" && kind != "model" && kind != "pause" && kind != "model-resume") return 4;
  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");
  gazebo::transport::SubscriberPtr stats;
  if (kind == "pause") stats=node->Subscribe("~/world_stats",on_stats);
  auto publisher=node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
  if (!publisher->WaitForConnection(gazebo::common::Time(2,0))) {
    gazebo::client::shutdown(); return 2;
  }
  gazebo::msgs::WorldControl control;
  if (kind == "pause") control.set_pause(true);
  else if (kind == "all") control.mutable_reset()->set_all(true);
  else if (kind == "time") control.mutable_reset()->set_time_only(true);
  else control.mutable_reset()->set_model_only(true);
  if (kind == "model-resume") control.set_pause(false);
  publisher->Publish(control);
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while(publisher->GetOutgoingCount() && std::chrono::steady_clock::now()<deadline) {
    publisher->SendMessage();std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool sent=publisher->GetOutgoingCount()==0;
  if (kind == "pause") {
    auto pause_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (!paused_observed.load() && std::chrono::steady_clock::now()<pause_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    sent=sent && paused_observed.load();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  gazebo::client::shutdown();return sent?0:3;
}
