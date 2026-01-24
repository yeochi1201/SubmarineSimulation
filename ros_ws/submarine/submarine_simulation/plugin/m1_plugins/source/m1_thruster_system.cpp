#include "m1_thruster_system.hpp"

#include <gz/sim/Entity.hh>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/plugin/Register.hh>

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
    model.EachLink(_ecm, [&](const ign::Entity &linkEnt)->
    {
      auto nameComp = _ecm.Component<ign::components::Name>(linkEnt);
    })
  }
}