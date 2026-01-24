#include "m1_thruster_system.hpp"

#include <atomic>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/plugin/Register.hh>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>

namespace ign = ignition::gazebo;
using Vector3d = ignition::math::Vector3d;

namespace m1_plugins
{
  ThrusterSystem::ThrusterSystem() = default;

  ThrusterSystem::~ThrusterSystem()
  {
    running_.store(false, std::memory_order_relaxed);
    if(spin_thread_.joinable())
      spin_thread_.join();
  }

  double ThrusterSystem::Clamp(double v, double lo, double hi)
  {
    if( v < lo ) return lo;
    if (v > hi ) return hi;
    return v;
  }

  static ignition::math::Vector3d ParseAxis(const std::string &axisStr)
  {
    double ax = 1.0, ay = 0.0, az = 0.0;

    (void)std::sscanf(axisStr.c_str(), "%lf %lf %lf", &ax, &ay, &az);
    Vector3d axis(ax, ay, az);
    if(axis.Length() < 1e-9)
      axis = Vector3d(1, 0, 0);
    axis.Normalize();

    return axis;
  }

  void ThrusterSystem::Configure( const ign::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        ign::EntityComponentManager &_ecm,
                        ign::EventManager &_eventMgr)
  {
    ign::Model model(_entity);

    //Read SDF Param
    link_name_  = _sdf->Get<std::string>("link_name", "bask_link").first;
    topic_      = _sdf->Get<std::string>("topic", "/m1/throttle").first;
    max_thrust_ = _sdf->Get<double>("max_thrust", 1e6).first;

    const std::string axisStr = _sdf->Get<std::string>("axis", "1 0 0").first;
    axis_body_ = ParseAxis(axisStr);

    //Find Target Link Entity By name
    link_entity_ = ign::kNullEntity;
    const auto links = model.Links(_ecm);
    for(const auto &linkEnt : links)
    {
      auto nameComp = _ecm.Component<ign::components::Name>(linkEnt);
      if(nameComp && nameComp->Data() == link_name_)
      {
        link_entity_ = linkEnt;
        break;
      }
    }

    if(link_entity_ == ign::kNullEntity)
    {
      ignerr << "[ThrusterSystem] Link Not Found: " << link_name_ << "\n";
      return;
    }

    // ROS2 Init
    if(!rclcpp::ok())
    {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("m1_thruster_system");
    sub_ = node_->create_subscription<std_msgs::msg::Float64>(topic_, 
      rclcpp::QoS(10), [&](const std_msgs::msg::Float64::SharedPtr msg)
    {
      double t = Clamp(msg->data, -1.0, -1.0);
      throttle_.store(t, std::memory_order_relaxed);
    });

    //Spring ROS Callbacks
    running_.store(true, std::memory_order_relaxed);
    spin_thread_ = std::thread([this]()
    {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node_);
      
      while(running_.load(std::memory_order_relaxed))
      {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }

      exec.remove_node(node_);
    });

    ignmsg  << "[ThrusterSystem] Configured"
            << " link = " << link_name_
            << " topic = " << topic_
            << " axis_body = " << axis_body_
            << " max_thrust = " << max_thrust_
            << "\n";
  }

  void ThrusterSystem::PreUpdate( const ign::UpdateInfo &_info, 
                                  ign::EntityComponentManager &_ecm)
  {
    if(_info.paused || link_entity_ == ign::kNullEntity) return;

    const double t = throttle_.load(std::memory_order_relaxed);
    if(std::abs(t) < 1e-9) return;

    // Body Frame Force
    const Vector3d f_body = axis_body_ * (t * max_thrust_);

    // Convert to world frame using current world pose rotation
    auto poseComp = _ecm.Component<ign::components::WorldPose>(link_entity_);
    if(poseComp) return;

    const auto poseW = poseComp->Data();
    const Vector3d f_world = poseW.Rot().RotateVector(f_body);

    // Apply Force
    ign::Link link(link_entity_);
    link.AddWorldForce(_ecm, f_world);
  }
}

//Plugin Regist
IGNITION_ADD_PLUGIN
(
  m1_plugins::ThrusterSystem,
  ignition::gazebo::System,
  m1_plugins::ThrusterSystem::ISystemConfigure,
  m1_plugins::ThrusterSystem::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(m1_plugins::ThrusterSystem, "m1_plugins::ThrusterSystem")