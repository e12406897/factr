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


# Each leader/follower pair (left, right, sim_left, sim_right) always runs both ends on
# the same machine, whichever machine that is -- so bind addresses use loopback, not a
# real network IP. Loopback is isolated per machine (never collides with another
# machine's loopback), so any number of pairs can run simultaneously, whether on one
# shared machine or spread across several, as long as each pair keeps its own ports
# distinct (which they already do below). Real network IPs would only be needed if a
# single pair's leader and follower had to run on two different machines -- not the
# case here.
franka_bridge_loopback_ip = "127.0.0.2"


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

# Distinct port sets per side so both sim arms can run simultaneously later for a
# bimanual sim setup (see mujoco_sim.py's --side and factr_teleop's franka_sim_left.yaml /
# franka_sim_right.yaml).
franka_sim_right_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_bridge_loopback_ip}:3099",
    "joint_torque_sub": f"tcp://{franka_bridge_loopback_ip}:3087",
    "raw_joint_torque_sub": f"tcp://{franka_bridge_loopback_ip}:3086",
    "joint_pos_cmd_pub": f"tcp://{franka_bridge_loopback_ip}:2098",

}

franka_sim_left_zmq_addresses = {
    "joint_state_sub":  f"tcp://{franka_bridge_loopback_ip}:5099",
    "joint_torque_sub": f"tcp://{franka_bridge_loopback_ip}:5087",
    "raw_joint_torque_sub": f"tcp://{franka_bridge_loopback_ip}:5086",
    "joint_pos_cmd_pub": f"tcp://{franka_bridge_loopback_ip}:4098",
}