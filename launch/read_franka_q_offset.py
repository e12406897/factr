"""One-shot: read the real Franka's current joint positions (q) from
franka_robot_state_broadcaster and print the offset against a target array you define.

Usage:
    python3 launch/read_franka_q_offset.py --target 0 0 0 -1.57 0 1.57 0.785
"""
import sys
from pathlib import Path
from typing import List

import numpy as np
import rclpy
import tyro
from franka_msgs.msg import FrankaRobotState
from rclpy.node import Node

_SRC_FACTR = Path(__file__).parent.parent / "src" / "factr"
sys.path.insert(0, str(_SRC_FACTR))


def main(
    target: List[float],
    robot_state_topic: str = "/franka_robot_state_broadcaster/robot_state",
    num_arm_joints: int = 7,
) -> None:
    assert len(target) == num_arm_joints, f"--target needs {num_arm_joints} entries"
    target_q = np.array(target, dtype=np.float64)

    rclpy.init()
    node = Node("read_franka_q_offset")
    received = {}

    def _on_state(msg: FrankaRobotState) -> None:
        received["q"] = np.array(msg.q[:num_arm_joints], dtype=np.float64)

    node.create_subscription(FrankaRobotState, robot_state_topic, _on_state, 10)

    node.get_logger().info(f"Waiting for one message on {robot_state_topic} ...")
    while rclpy.ok() and "q" not in received:
        rclpy.spin_once(node, timeout_sec=1.0)

    current_q = received["q"]
    offset = target_q - current_q

    np.set_printoptions(precision=4, suppress=True)
    print(f"current q:      {current_q}")
    print(f"target q:       {target_q}")
    print(f"offset (target - current): {offset}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    tyro.cli(main)