/**
 * @file pid_temperature_node.cpp
 * @brief Example: PID temperature controller as a ROS 2 lifecycle node.
 *
 * Topics:
 *   /pid_temperature_controller/setpoint      [std_msgs/Float64]  target °C
 *   /pid_temperature_controller/measurement   [std_msgs/Float64]  sensor °C
 *   /pid_temperature_controller/control_output [std_msgs/Float64] heater duty [0,100]
 *
 * Parameters (set via ros2 param or YAML file):
 *   sample_time_s  (double, default 0.05)   control loop period
 *   Kp             (double, default 2.0)
 *   Ki             (double, default 0.1)
 *   Kd             (double, default 0.05)
 *   u_min          (double, default   0.0)  heater off
 *   u_max          (double, default 100.0)  heater full
 *
 * Build + run:
 *   colcon build --packages-select ctrl_toolbox_ros2
 *   source install/setup.bash
 *   ros2 lifecycle set /pid_temperature_controller configure
 *   ros2 lifecycle set /pid_temperature_controller activate
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ctrl_toolbox_ros2/controller_node.hpp"

// Controller Toolbox headers (available after find_package(ControllerToolbox))
#include "controller_toolbox/DiscretePID.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Factory: runs inside on_configure(); can read/declare any ROS 2 params.
    auto factory = [](rclcpp_lifecycle::LifecycleNode & node)
        -> std::shared_ptr<ctrl::DiscretePID>
    {
        ctrl::DiscretePIDParams p;
        p.Ts   = node.get_parameter("sample_time_s").as_double();
        p.Kp   = node.declare_parameter<double>("Kp",    2.0);
        p.Ki   = node.declare_parameter<double>("Ki",    0.1);
        p.Kd   = node.declare_parameter<double>("Kd",    0.05);
        p.Kb   = 1.0;   // anti-windup back-calculation gain
        p.N    = 10.0;  // derivative filter coefficient
        p.uMin = node.declare_parameter<double>("u_min",   0.0);
        p.uMax = node.declare_parameter<double>("u_max", 100.0);
        return std::make_shared<ctrl::DiscretePID>(p);
    };

    auto node = std::make_shared<
        ctrl_toolbox_ros2::ControllerNode<ctrl::DiscretePID>>(
            "pid_temperature_controller", factory);

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
