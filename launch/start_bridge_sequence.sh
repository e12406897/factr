#!/bin/bash
# Sequential steps for the "bridge" tmux window: spawn joint_trajectory_controller,
# send a one-shot move to the start pose, then start the ZMQ<->ROS2 bridge (long-running).
# Split into its own file (rather than inlined into tmux send-keys) to avoid nested-quoting
# issues with the YAML-ish ros2 topic pub argument.
set -e

# Franka's own "home" joint configuration (unchanged default -- see franka.launch.py /
# libfranka's own homing pose), used whenever no initial pose is given.
HOME_POSE="0,0,0,-1.57,0,1.57,0.785"

SIDE="${1:-}"
INITIAL_POSE="${2:-$HOME_POSE}"
SAVE_LAUNCH="${3:-False}"

USAGE="Usage: $0 {left|right} [j1,j2,j3,j4,j5,j6,j7] [True|False]"

# Accept an optional surrounding [ ] -- e.g. copy-pasted straight from a config file or a
# previous log line -- as well as the bare comma-separated form.
INITIAL_POSE="${INITIAL_POSE#\[}"
INITIAL_POSE="${INITIAL_POSE%\]}"

if [[ "$SIDE" != "left" && "$SIDE" != "right" ]]; then
    echo "$USAGE"
    exit 1
fi

if [[ "$SAVE_LAUNCH" != "True" && "$SAVE_LAUNCH" != "False" ]]; then
    echo "Error: save_launch must be True or False"
    echo "$USAGE"
    exit 1
fi

# Exactly 7 comma-separated numbers (int or float, optional sign) -- fed straight into
# the JointTrajectory positions array below, so catch a malformed value here rather
# than sending a bad command to the robot.
FLOAT='[+-]?[0-9]+(\.[0-9]+)?'
if ! [[ "$INITIAL_POSE" =~ ^${FLOAT}(,${FLOAT}){6}$ ]]; then
    echo "Error: initial pose must be exactly 7 comma-separated numbers (no spaces), got '$INITIAL_POSE'"
    echo "$USAGE"
    exit 1
fi
INITIAL_POSE_LIST="${INITIAL_POSE//,/, }"

CONFIG_FILE="franka_${SIDE}.yaml"

cd /factr
source /opt/ros/humble/setup.bash
source /factr/install/setup.bash

echo "[1/3] Spawning joint_trajectory_controller..."
ros2 run controller_manager spawner joint_trajectory_controller

echo "[2/3] Sending one-shot move to start pose (${INITIAL_POSE_LIST})..."
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [${INITIAL_POSE_LIST}], time_from_start: {sec: 4, nanosec: 0}}]}"

echo "[3/3] Starting franka_single_arm.launch.py bridge..."
if [[ "$SAVE_LAUNCH" == "True" ]]; then
    python3 launch/franka_single_arm.launch.py \
        --side "$SIDE" \
        --config-file "$CONFIG_FILE" \
        --save-launch
else
    python3 launch/franka_single_arm.launch.py \
        --side "$SIDE" \
        --config-file "$CONFIG_FILE"
fi
