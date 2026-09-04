#!/bin/bash
# Sequential steps for the "bridge" tmux window: spawn joint_trajectory_controller,
# send a one-shot move to the start pose, then start the ZMQ<->ROS2 bridge (long-running).
# Split into its own file (rather than inlined into tmux send-keys) to avoid nested-quoting
# issues with the YAML-ish ros2 topic pub argument.
set -e

cd /factr
source /opt/ros/humble/setup.bash
source /factr/install/setup.bash

echo "[1/3] Spawning joint_trajectory_controller..."
ros2 run controller_manager spawner joint_trajectory_controller

echo "[2/3] Sending one-shot move to start pose..."
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [0, 0, 0, -1.57, 0, 1.57, 0.785], time_from_start: {sec: 4, nanosec: 0}}]}"

echo "[3/3] Starting franka_ros2_follower.py bridge..."
python3 launch/franka_ros2_follower.py --name left
