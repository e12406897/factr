// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Copied from franka_ros2 v0.1.0
// franka_example_controllers/src/joint_impedance_example_controller.cpp.
// Control law (k_gains / d_gains PD on effort, filtered velocity) unchanged. Changes:
//  - the built-in periodic demo motion of joints 4/5 replaced by a goal from
//    ~/joint_target (std_msgs/Float64MultiArray, 7 joint positions [rad]); the goal
//    starts at the joint positions on activation, so activating never moves the robot.
//  - the goal is low-pass filtered each cycle (filter_params_, same as the Cartesian
//    impedance controller), so 20-500 Hz teleop targets don't produce torque steps.

#include <factr_controllers/joint_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace factr_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  const Vector7d q_goal = *target_buffer_.readFromRT();
  q_d_ = filter_params_ * q_goal + (1.0 - filter_params_) * q_d_;

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_d_ - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  target_buffer_.initRT(Vector7d::Zero());
  sub_joint_target_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/joint_target", rclcpp::SystemDefaultsQoS(),
      std::bind(&JointImpedanceController::jointTargetCallback, this, std::placeholders::_1));
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  q_d_ = q_;
  target_buffer_.initRT(q_);
  dq_filtered_.setZero();
  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void JointImpedanceController::jointTargetCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != static_cast<size_t>(num_joints)) {
    RCLCPP_WARN(get_node()->get_logger(), "Discarding joint target of size %zu (expected %d).",
                msg->data.size(), num_joints);
    return;
  }
  Vector7d target;
  for (int i = 0; i < num_joints; ++i) {
    if (!std::isfinite(msg->data[i])) {
      RCLCPP_WARN(get_node()->get_logger(), "Discarding non-finite joint target.");
      return;
    }
    target(i) = msg->data[i];
  }
  target_buffer_.writeFromNonRT(target);
}

}  // namespace factr_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(factr_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)
