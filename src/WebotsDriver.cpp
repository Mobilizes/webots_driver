#include "webots_driver/WebotsDriver.hpp"

#include <functional>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include "jitsuyo/cli.hpp"
#include "kansei_interfaces/msg/status.hpp"
#include "keisan/keisan.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"

std::string motorNames[20] = {
  "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
  "ArmLowerL", "PelvYR",    "PelvYL",    "PelvR",     "PelvL",
  "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
  "AnkleL",    "FootR",     "FootL",     "Neck",      "Head"
};

// std::string jointNames[20] = {
//   "right_shoulder_pitch", "left_shoulder_pitch", "right_shoulder_roll", "left_shoulder_roll", "right_elbow",
//   "left_elbow", "right_hip_yaw", "left_hip_yaw", "right_hip_roll", "left_hip_roll",
//   "right_hip_pitch", "left_hip_pitch", "right_knee", "left_knee", "right_ankle_pitch",
//   "left_ankle_pitch", "right_ankle_roll", "left_ankle_roll", "neck_yaw", "neck_pitch"
// };

using keisan::literals::operator""_pi;

double degToRad(double degree) { return degree * 1_pi / 180.0; }
double radToDeg(double radian) { return radian * 180.0 / 1_pi; }

namespace webots_driver {
void WebotsDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {
  
  robot = new webots::Robot;

  for (int i = 0; i < 20; ++i)
    joints.push_back(tachimawari::joint::Joint(i + 1, 0.0));

  jointsOffset = {
    0.0_pi, 0.0_pi, 0.0_pi, 0.0_pi, -0.5_pi,
    0.5_pi, 0.0_pi, 0.0_pi, 0.0_pi, 0.0_pi,
    0.0_pi, 0.0_pi, 0.0_pi, 0.0_pi, 0.0_pi,
    0.0_pi, 0.0_pi, 0.0_pi, 0.0_pi, 0.0_pi,
  };

  jointsLowerLimit.resize(20);
  jointsUpperLimit.resize(20);

  for (int i = 0; i < 20; ++i) {
    motors[i] = robot->getMotor(motorNames[i]);
    if (!motors[i]) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize motor: %s", motorNames[i]);
      return;
    }
    motors[i]->setPosition(jointsOffset[i]);
    motors[i]->setVelocity(motors[i]->getMaxVelocity());
    jointsLowerLimit[i] = motors[i]->getMinPosition();
    jointsUpperLimit[i] = motors[i]->getMaxPosition();
  }

  current_joints_subscription = node->create_subscription<CurrentJoints>(
    "/joint/current_joints", rclcpp::SensorDataQoS().reliable(),
    std::bind(&WebotsDriver::currentJointsCallback, this, std::placeholders::_1));

  measurement_status_subscription = node->create_subscription<MeasurementStatus>(
    "/measurement/status", rclcpp::SensorDataQoS().reliable(),
    std::bind(&WebotsDriver::measurementStatusCallback, this, std::placeholders::_1));
}

void WebotsDriver::adjustInit(double & position, const uint8_t & id) {
  position += jointsOffset[id - 1];
  position = keisan::clamp(position, jointsLowerLimit[id - 1], jointsUpperLimit[id - 1]);
}

void WebotsDriver::currentJointsCallback(const CurrentJoints::SharedPtr msg) {
  for (auto joint : msg->joints) {
    joints[(int)joint.id - 1] = tachimawari::joint::Joint(joint.id, joint.position);
  }
}

void WebotsDriver::measurementStatusCallback(const MeasurementStatus::SharedPtr msg) {
  keisan::Angle<double> roll = keisan::make_degree(msg->orientation.roll);
  keisan::Angle<double> pitch = keisan::make_degree(msg->orientation.pitch);
  keisan::Angle<double> yaw = keisan::make_degree(msg->orientation.yaw);

  orientation = keisan::Euler<double>(roll, pitch, yaw);
}

void WebotsDriver::step() {
  jitsuyo::clear();
  for (auto joint : joints) {
    double position = degToRad(joint.get_position());
    adjustInit(position, joint.get_id());
    std::cout << (int)joint.get_id() << ": " << radToDeg(position) << "\n";
    motors[joint.get_id() - 1]->setPosition(position);
  }
  std::cout << "RPY : " << \
    orientation.roll.degree() << " " << \
    orientation.pitch.degree() << " " << \
    orientation.yaw.degree() << "\n";
}
}  // namespace webots_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(webots_driver::WebotsDriver,
                       webots_ros2_driver::PluginInterface)
