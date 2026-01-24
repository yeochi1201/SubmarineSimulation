#include "m1_thruster_system.hpp"

#include <ignition/common/Console.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/plugin/Register.hh>

namespace m1_plugins
{
  ThrusterSystem::ThrusterSystem() = default;

  ThrusterSystem::~ThrusterSystem()
  {
    running_.store(false, std::memory_order_relaxed);
    if(spin_thread_.joinable())
      spin_thread_.join();
  }

}