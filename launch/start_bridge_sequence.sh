#!/bin/bash
# Sequential steps for the "bridge" tmux window: spawn joint_trajectory_controller,
# send a one-shot move to the start pose, switch to the requested controller (if not the
# trajectory controller), then start the ZMQ<->ROS2 bridge (long-running).
# Split into its own file (rather than inlined into tmux send-keys) to avoid nested-quoting
# issues with the YAML-ish ros2 topic pub argument.
set -e

USAGE="Usage: $0 {left|right} [True|False] [trajectory_controller|joint_impedance_controller|cartesian_impedance_controller]"

SIDE="${1:-}"
SAVE_LAUNCH="${2:-False}"
CONTROLLER="${3:-trajectory_controller}"

if [[ "$SIDE" != "left" && "$SIDE" != "right" ]]; then
    echo "$USAGE"
    exit 1
fi

if [[ "$SAVE_LAUNCH" != "True" && "$SAVE_LAUNCH" != "False" ]]; then
    echo "Error: save_launch must be True or False"
    echo "$USAGE"
    exit 1
fi

case "$CONTROLLER" in
    trajectory_controller)
        ROS_CONTROLLER="joint_trajectory_controller" ;;
    joint_impedance_controller)
        ROS_CONTROLLER="joint_impedance_controller"
        CONTROLLER_TYPE="factr_controllers/JointImpedanceController" ;;
    cartesian_impedance_controller)
        ROS_CONTROLLER="cartesian_impedance_controller"
        CONTROLLER_TYPE="factr_controllers/CartesianImpedanceController" ;;
    *)
        echo "Error: unknown controller '$CONTROLLER'"
        echo "$USAGE"
        exit 1 ;;
esac

CONFIG_FILE="franka_${SIDE}.yaml"

cd /factr
source /opt/ros/humble/setup.bash
source /factr/install/setup.bash

echo "[1/4] Spawning joint_trajectory_controller..."
ros2 run controller_manager spawner joint_trajectory_controller

echo "[2/4] Sending one-shot move to start pose..."
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [0, 0, 0, -1.57, 0, 1.57, 0.785], time_from_start: {sec: 4, nanosec: 0}}]}"

if [[ "$ROS_CONTROLLER" != "joint_trajectory_controller" ]]; then
    echo "[3/4] Switching to $ROS_CONTROLLER..."
    # Let the start-pose move finish first (4 s trajectory).
    sleep 5
    ros2 run controller_manager spawner "$ROS_CONTROLLER" \
        --controller-type "$CONTROLLER_TYPE" \
        --param-file "/factr/src/factr/factr_controllers/config/${ROS_CONTROLLER}.yaml" \
        --inactive
    # One strict switch: the effort interfaces are handed over within the same update
    # cycle, never left without an active controller.
    SWITCH_RESULT=$(ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
        "{activate_controllers: [$ROS_CONTROLLER], deactivate_controllers: [joint_trajectory_controller], strictness: 2}")
    echo "$SWITCH_RESULT"
    if ! grep -q "ok=True" <<< "$SWITCH_RESULT"; then
        echo "Error: switching to $ROS_CONTROLLER failed -- joint_trajectory_controller stays active, bridge not started."
        exit 1
    fi
else
    echo "[3/4] Keeping joint_trajectory_controller."
fi

echo "[4/4] Starting franka_ros2_follower.py bridge ($ROS_CONTROLLER)..."
if [[ "$SAVE_LAUNCH" == "True" ]]; then
    python3 launch/franka_ros2_follower.py \
        --side "$SIDE" \
        --config-file "$CONFIG_FILE" \
        --controller "$ROS_CONTROLLER" \
        --save-launch
else
    python3 launch/franka_ros2_follower.py \
        --side "$SIDE" \
        --config-file "$CONFIG_FILE" \
        --controller "$ROS_CONTROLLER"
fi
