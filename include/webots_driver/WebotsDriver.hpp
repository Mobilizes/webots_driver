#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"

namespace webots_driver {
class WebotsDriver : public webots_ros2_driver::PluginInterface {
public:
  using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;

  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;
  
private:
  void currentJointsCallback(const CurrentJoints::SharedPtr msg);
  void adjustInit(double & position, const uint8_t & id);

  rclcpp::Subscription<CurrentJoints>::SharedPtr current_joints_subscription;

  std::vector<tachimawari::joint::Joint> joints;
  std::vector<double> jointsOffset;
  std::vector<double> jointsLowerLimit;
  std::vector<double> jointsUpperLimit;

  webots::Robot *robot;
  webots::Motor *motors[20];
};
}  // namespace webots_driver
#endif
