// Copyright (c) 2023 Franka Robotics GmbH
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

// Copied from franka_ros2 (jazzy branch)
// franka_example_controllers/include/franka_example_controllers/fr3/cartesian_impedance_example_controller.hpp
// and ported to franka_ros2 v0.1.0 / ROS 2 Humble -- see the .cpp for the list of changes.

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_buffer.h>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include <controller_interface/controller_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"

namespace factr_controllers {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  struct TargetPose {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
  };

  static constexpr int num_joints{7};
  static constexpr int num_cartesian_dof{6};

  struct CartesianGains {
    Eigen::Matrix<double, num_cartesian_dof, num_cartesian_dof> stiffness{
        Eigen::Matrix<double, num_cartesian_dof, num_cartesian_dof>::Zero()};
    Eigen::Matrix<double, num_cartesian_dof, num_cartesian_dof> damping{
        Eigen::Matrix<double, num_cartesian_dof, num_cartesian_dof>::Zero()};
  };

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  std::string arm_id_;

  Eigen::Matrix<double, num_joints, 1> q_{Eigen::Matrix<double, num_joints, 1>::Zero()};
  Eigen::Matrix<double, num_joints, 1> dq_{Eigen::Matrix<double, num_joints, 1>::Zero()};

  double filter_params_{0.005};

  Eigen::Matrix<double, num_cartesian_dof, num_cartesian_dof> cartesian_stiffness_;
  Eigen::Matrix<double, num_cartesian_dof, num_cartesian_dof> cartesian_damping_;
  double nullspace_stiffness_{20.0};

  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;

  Eigen::Matrix<double, num_joints, 1> q_d_nullspace_;

  realtime_tools::RealtimeBuffer<TargetPose> target_pose_buffer_;
  realtime_tools::RealtimeBuffer<CartesianGains> cartesian_gains_buffer_;
  realtime_tools::RealtimeBuffer<double> nullspace_stiffness_buffer_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  /// @brief Callback for incoming equilibrium pose messages.
  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /// @brief Callback for live parameter updates (nullspace_stiffness).
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
      const std::vector<rclcpp::Parameter>& parameters);

  /// @brief Reads joint positions and velocities from loaned state interfaces.
  void readJointState();

  /// @brief Current end-effector pose (O_T_EE) from the robot model.
  std::pair<Eigen::Quaterniond, Eigen::Vector3d> getCurrentOrientationAndTranslation();

  /// @brief Computes the 6D Cartesian pose error between current and desired pose.
  Eigen::Matrix<double, num_cartesian_dof, 1> computeError(const Eigen::Vector3d& position,
                                                           const Eigen::Quaterniond& orientation,
                                                           const Eigen::Affine3d& transform) const;

  /// @brief Builds stiffness and damping matrices from a 6-vector of stiffness values.
  static CartesianGains buildGains(const std::array<double, num_cartesian_dof>& k);
};

}  // namespace factr_controllers
