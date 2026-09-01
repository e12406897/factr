# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------

sim_desktop_ip_address = "172.16.0.9"
# Actual network IPs of the Franka control units. No longer used directly below: the
# original architecture assumed an external libfranka/ZMQ driver running ON these robot
# control units, but that driver is not part of this repo. `franka_ros2_follower.py`
# fills that role instead, running in the same devcontainer as the leader (localhost),
# not on the robot's own control unit — so the ZMQ addresses below bind/connect via
# `franka_bridge_loopback_ip` instead of these.
franka_left_ip_address = "172.16.0.1"
franka_right_ip_address = "172.16.0.3"
franka_sim_ip_address = "127.0.0.1"
# Leader and franka_ros2_follower.py both run inside the same devcontainer.
franka_bridge_loopback_ip = "127.0.0.1"


franka_right_real_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_bridge_loopback_ip}:3099",
    "joint_torque_sub": f"tcp://{franka_bridge_loopback_ip}:3087",
    "raw_joint_torque_sub": f"tcp://{franka_bridge_loopback_ip}:3086",
    "joint_pos_cmd_pub": f"tcp://{franka_bridge_loopback_ip}:2098",

}

franka_left_real_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_bridge_loopback_ip}:5099",
    "joint_torque_sub": f"tcp://{franka_bridge_loopback_ip}:5087",
    "raw_joint_torque_sub": f"tcp://{franka_bridge_loopback_ip}:5086",
    "joint_pos_cmd_pub": f"tcp://{franka_bridge_loopback_ip}:4098",

}

franka_sim_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_sim_ip_address}:16099",
    "joint_torque_sub": f"tcp://{franka_sim_ip_address}:16087",
    "joint_pos_cmd_pub": f"tcp://{franka_sim_ip_address}:16098",
    "lpass_filter_sub": f"tcp://{franka_sim_ip_address}:16080",
    "raw_joint_torque_sub": f"tcp://{franka_sim_ip_address}:16070"
}