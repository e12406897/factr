#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

FRANKA_ROS2_VERSION=v0.1.0

if [ ! -d src/franka_ros2 ]; then
    git clone --branch "$FRANKA_ROS2_VERSION" --depth 1 \
        https://github.com/frankarobotics/franka_ros2.git src/franka_ros2

    # franka_ros2's controllers.yaml keys parameters by each node's unnamespaced
    # name (e.g. "controller_manager:"). franka_dual_arm.launch.py pushes each
    # robot into its own ROS2 namespace (left/right), so the resolved node names
    # become e.g. "/left/controller_manager" and no longer match those keys —
    # every parameter under them (including required ones, like
    # joint_state_broadcaster's "type") silently fails to load. Rewriting each
    # top-level key to the ROS2 "/**/<name>:" wildcard form matches the node
    # regardless of namespace while keeping each node's own distinct key (no
    # collision risk, unlike merging everything under one shared "/**:" block).
    sed -i -E 's/^([a-zA-Z_][a-zA-Z0-9_]*):$/\/**\/\1:/' \
        src/franka_ros2/franka_bringup/config/controllers.yaml
fi

rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys libfranka

colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DCMAKE_CXX_FLAGS=-I/home/asl_team/libfranka/include
