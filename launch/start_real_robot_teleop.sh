#!/bin/bash
# One-command launcher for the real-hardware teleop stack:
#   0. kill leftover processes/tmux session from previous runs
#   1. (tmux window "hardware") franka.launch.py
#   2. (tmux window "bridge", after /controller_manager is up) spawn
#      joint_trajectory_controller -> one-shot move to start pose ->
#      franka_ros2_follower.py bridge (see start_bridge_sequence.sh)
#
# Usage: bash launch/start_real_robot_teleop.sh
# Detach from tmux with Ctrl-b d (leaves everything running); reattach with:
#   tmux attach -t factr
set -e

SESSION="factr"

# One-time, idempotent setup of window-switching shortcuts for a second, plain terminal
# tab (VS Code's integrated terminal intercepts tmux's Ctrl-b prefix, so switching via
# tmux keybindings from inside the attached terminal itself doesn't work reliably there —
# these aliases let you switch by running a short command from a separate tab instead).
if ! grep -qF "alias sw-hw=" ~/.bashrc 2>/dev/null; then
    echo "alias sw-hw='tmux select-window -t ${SESSION}:hardware'" >> ~/.bashrc
fi
if ! grep -qF "alias sw-br=" ~/.bashrc 2>/dev/null; then
    echo "alias sw-br='tmux select-window -t ${SESSION}:bridge'" >> ~/.bashrc
fi

echo "Killing leftover processes from previous runs..."
tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -9 -f "ros2_control_node" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pkill -9 -f "franka_gripper_node" 2>/dev/null || true
pkill -9 -f "franka_ros2_follower.py" 2>/dev/null || true
pkill -9 -f "spawner joint_trajectory_controller" 2>/dev/null || true
sleep 1

cd /factr
source /opt/ros/humble/setup.bash
source /factr/install/setup.bash

tmux new-session -d -s "$SESSION" -n hardware
tmux send-keys -t "$SESSION":hardware \
    "cd /factr && source /opt/ros/humble/setup.bash && source /factr/install/setup.bash && ros2 launch franka_bringup franka.launch.py robot_ip:=franka" \
    C-m

echo "Waiting for /controller_manager to come up (max 60s)..."
for i in $(seq 1 60); do
    if ros2 service list 2>/dev/null | grep -q "/controller_manager/list_controllers"; then
        echo "/controller_manager is up."
        break
    fi
    if [ "$i" -eq 60 ]; then
        echo "WARNING: /controller_manager did not come up within 60s — check the" \
             "'hardware' tmux window for errors (tmux attach -t $SESSION). Continuing anyway."
    fi
    sleep 1
done

tmux new-window -t "$SESSION" -n bridge
tmux send-keys -t "$SESSION":bridge "bash /factr/launch/start_bridge_sequence.sh" C-m

echo "Attaching to tmux session '$SESSION' (windows: hardware, bridge)."
echo "Ctrl-b keybindings don't work reliably in VS Code's terminal — instead, open a"
echo "second, plain terminal tab and run 'sw-hw' / 'sw-br' there to switch what shows"
echo "here (open a new tab first so the aliases just added to ~/.bashrc are loaded)."
tmux attach-session -t "$SESSION"
