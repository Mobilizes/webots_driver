#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "kansei_interfaces/msg/status.hpp"
#include "keisan/keisan.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"

#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace webots_driver {
class WebotsDriver : public webots_ros2_driver::PluginInterface {
public:
  using MeasurementStatus = kansei_interfaces::msg::Status;
  using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;

  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;
  
private:
  void adjustInit(double & position, const uint8_t & id);
  void currentJointsCallback(const CurrentJoints::SharedPtr msg);
  void measurementStatusCallback(const MeasurementStatus::SharedPtr msg);
  void stepMotion();
  void stepVision();

  rclcpp::Subscription<CurrentJoints>::SharedPtr current_joints_subscription;
  rclcpp::Subscription<MeasurementStatus>::SharedPtr measurement_status_subscription;

  keisan::Euler<double> orientation;

  std::vector<tachimawari::joint::Joint> joints;
  std::vector<double> joints_offset;
  std::vector<double> joints_lower_limit;
  std::vector<double> joints_upper_limit;

  webots::Camera *camera;
  webots::Motor *motors[20];
  webots::Supervisor *robot;
};
}  // namespace webots_driver
#endif
