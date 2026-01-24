#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/math/Vector3.hh>


namespace m1_plugins
{
    /// \brief Single Thruster (surge) force Applicator
    /// - Subscribes ROS2 Topic (std_msgs/Float64) throttle [-1, 1]
    /// - Applies Force to Target Link each Simulation Tick
    ///     F_body = axis_body * throttle * max_thrust
    ///     F_world = R_world_from_body * F_body
    ///     Link::AddWorldForce(ecm, F_world)
    class ThrusterSystem final
     :  public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
    {
    public:
        ThrusterSystem();
        ~ThrusterSystem() override;

        void Configure( const ignition::gazebo::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        ignition::gazebo::EntityComponentManager &_ecm,
                        ignition::gazebo::EventManager &_eventMgr) override;
        
        void PreUpdate( const ignition::gazebo::UpdateInfo &_info,
                        ignition::gazebo::EntityComponentManager &_ecm) override;
    
    private:
        // --- Helper ---
        static ignition::math::Vector3d ParseAxis(const std::string &axisStr);
        static double Clamp(double v, double lo, double hi);

        // --- SDF Params ---
        std::string link_name_{"base_link"};
        std::string topic_{"m1/thrttle"};
        ignition::math::Vector3d axis_body_{1, 0, 0}; //normalized
        double max_thrust_{1e6};

        // --- Gazebo Entities ---
        ignition::gazebo::Entity link_entity_{ignition::gazebo::kNullEntity};

        // --- ROS2 Side ---
        std::atomic<double> throttle_{0.0};
        std::atomic<bool> running_{false};

        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
        std::thread spin_thread_;
    };
}