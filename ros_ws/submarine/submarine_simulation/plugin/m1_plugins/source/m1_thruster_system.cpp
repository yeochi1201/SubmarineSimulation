// src/m1_thruster_system.cpp
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/WorldPose.hh>

#include <ignition/math/Vector3.hh>

#include <ignition/plugin/Register.hh>   // IGNITION_ADD_PLUGIN
#include <ignition/common/Console.hh>    // ignmsg/ignerr

namespace m1
{
class ThrusterSystem final
  : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
public:
  ThrusterSystem() = default;

  ~ThrusterSystem() override
  {
    running_.store(false);
    if (spin_thread_.joinable())
      spin_thread_.join();
  }

  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &) override
  {
    ignition::gazebo::Model model(_entity);

    // ---- Read SDF params ----
    link_name_  = _sdf->Get<std::string>("link_name", "base_link").first;
    topic_      = _sdf->Get<std::string>("topic", "/m1/throttle").first;
    max_thrust_ = _sdf->Get<double>("max_thrust", 1e6).first;

    // axis can be either "axis" as string "x y z" or as an <axis> element text.
    // We'll parse string form for simplicity/robustness.
    const std::string axisStr = _sdf->Get<std::string>("axis", "1 0 0").first;
    double ax = 1, ay = 0, az = 0;
    (void)std::sscanf(axisStr.c_str(), "%lf %lf %lf", &ax, &ay, &az);
    axis_body_ = ignition::math::Vector3d(ax, ay, az);

    if (axis_body_.Length() < 1e-9)
    {
      ignerr << "[ThrusterSystem] axis is zero vector. Using default (1,0,0)\n";
      axis_body_.Set(1, 0, 0);
    }
    axis_body_.Normalize();

    // ---- Find link entity by name ----
    link_entity_ = ignition::gazebo::kNullEntity;
    model.EachLink(_ecm, [&](const ignition::gazebo::Entity &linkEnt) -> bool
    {
      auto nameComp = _ecm.Component<ignition::gazebo::components::Name>(linkEnt);
      if (nameComp && nameComp->Data() == link_name_)
      {
        link_entity_ = linkEnt;
        return false; // stop iterating
      }
      return true;
    });

    if (link_entity_ == ignition::gazebo::kNullEntity)
    {
      ignerr << "[ThrusterSystem] Link not found: " << link_name_ << "\n";
      return;
    }

    // ---- ROS 2 init (safe if called multiple times) ----
    if (!rclcpp::ok())
    {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("m1_thruster_system");
    sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      topic_, rclcpp::QoS(10),
      [&](const std_msgs::msg::Float64::SharedPtr msg)
      {
        double t = msg->data;
        if (t >  1.0) t =  1.0;
        if (t < -1.0) t = -1.0;
        throttle_.store(t, std::memory_order_relaxed);
      });

    // Spin ROS in a small background thread
    running_.store(true, std::memory_order_relaxed);
    spin_thread_ = std::thread([this]()
    {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_);
      while (running_.load(std::memory_order_relaxed))
      {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      exec.remove_node(node_);
    });

    ignmsg << "[ThrusterSystem] Configured. link=" << link_name_
           << " topic=" << topic_
           << " max_thrust=" << max_thrust_
           << " axis_body=" << axis_body_ << "\n";
  }

  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused || link_entity_ == ignition::gazebo::kNullEntity)
      return;

    // Read throttle [-1, 1]
    const double t = throttle_.load(std::memory_order_relaxed);
    if (std::abs(t) < 1e-9)
      return; // micro-optimization

    // Force in body frame (N)
    ignition::math::Vector3d f_body = axis_body_ * (t * max_thrust_);

    // Need world pose for body->world rotation
    auto poseComp = _ecm.Component<ignition::gazebo::components::WorldPose>(link_entity_);
    if (!poseComp)
      return;

    const auto poseW = poseComp->Data();
    ignition::math::Vector3d f_world = poseW.Rot().RotateVector(f_body);

    // Apply force to link (World frame)
    ignition::gazebo::Link link(link_entity_);
    link.AddWorldForce(_ecm, f_world);
  }

private:
  std::string link_name_{"base_link"};
  std::string topic_{"/m1/throttle"};
  double max_thrust_{1e6};

  ignition::math::Vector3d axis_body_{1, 0, 0};

  ignition::gazebo::Entity link_entity_{ignition::gazebo::kNullEntity};

  std::atomic<double> throttle_{0.0};
  std::atomic<bool> running_{false};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  std::thread spin_thread_;
};
}  // namespace m1

// Fortress / Ignition Gazebo plugin registration
IGNITION_ADD_PLUGIN(
  m1::ThrusterSystem,
  ignition::gazebo::System,
  m1::ThrusterSystem::ISystemConfigure,
  m1::ThrusterSystem::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(m1::ThrusterSystem, "m1::ThrusterSystem")
