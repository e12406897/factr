from typing import Dict, List

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from franka_msgs.msg import FrankaRobotState
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FrankaRos2Follower(Node):
    """Bridges FACTR's ZMQ leader protocol to a real Franka robot running franka_ros2's
    `joint_trajectory_controller`, so the unmodified `FACTRTeleopFrankaZMQ` leader node
    can teleoperate real hardware without the external libfranka/ZMQ driver that the
    original FACTR ZMQ addresses assume.

    Structurally mirrors `MujocoFrankaFollower`, but the actuator backend is a real
    franka_ros2 `ros2_control` stack (ROS2 topics) instead of a MuJoCo physics loop.

    ZMQ direction (mirrors the real Franka driver / MujocoFrankaFollower):
      - subscribes (connects) to `joint_pos_cmd_pub`: leader arm position targets
      - publishes  (binds)    on `joint_state_sub`:      follower arm joint positions
      - publishes  (binds)    on `joint_torque_sub`:     follower arm external joint torque
      - publishes  (binds)    on `raw_joint_torque_sub`: same signal, unprocessed

    Unlike the sim, the real robot has no "rawer" signal than libfranka's own
    `tau_ext_hat_filtered` available, so both torque channels currently carry the same
    value. `torque_sign` exists because the sim needed an empirically-found `-1` to
    match `torque_feedback()`'s sign convention (see MujocoFrankaFollower.serve()) —
    that was compensating for a MuJoCo-internal constraint-force convention and is NOT
    assumed to also apply to the real robot's own tau_ext_hat_filtered. Verify the sign
    empirically (per-joint contact test) before trusting force-feedback direction, and
    adjust this constructor argument if needed.

    Prerequisites this class does NOT set up for you:
      - `franka.launch.py` already running against the real robot (hardware active).
      - `joint_trajectory_controller` spawned, e.g.:
            ros2 run controller_manager spawner joint_trajectory_controller
      - `robot_state_topic` verified via `ros2 topic list` once
        `franka_robot_state_broadcaster` is running — the default here is a best guess,
        not confirmed against your actual running system.
      - Running two robots at once additionally requires each robot's own
        `franka.launch.py`/`joint_trajectory_controller` to live in a distinct ROS2
        namespace (franka_ros2 v0.1.0 has no built-in namespace support for this) —
        that is a separate, still-open task, not solved by this bridge alone.

    Gripper force-feedback is not covered here (matches how the sim follower staged
    arm support before gripper support).
    """

    JOINT_NAMES: List[str] = [f"panda_joint{i}" for i in range(1, 8)]

    def __init__(
        self,
        zmq_addresses: Dict[str, str],
        num_arm_joints: int = 7,
        trajectory_topic: str = "/joint_trajectory_controller/joint_trajectory",
        robot_state_topic: str = "/franka_robot_state_broadcaster/robot_state",
        node_name: str = "factr_franka_ros2_follower",
        command_period_sec: float = 0.002,
        trajectory_point_duration_sec: float = 0.01,
        torque_sign: float = 1.0,
    ):
        super().__init__(node_name)
        self._num_arm_joints = num_arm_joints
        self._torque_sign = torque_sign

        self._trajectory_point_duration = Duration(
            sec=int(trajectory_point_duration_sec),
            nanosec=int((trajectory_point_duration_sec % 1.0) * 1e9),
        )
        self._trajectory_pub = self.create_publisher(
            JointTrajectory, trajectory_topic, 10
        )
        self._state_sub = self.create_subscription(
            FrankaRobotState, robot_state_topic, self._on_robot_state, 10
        )

        self._cmd_addr = zmq_addresses["joint_pos_cmd_pub"]
        self._cmd_sub = ZMQSubscriber(self._cmd_addr)
        self._state_pub = ZMQPublisher(zmq_addresses["joint_state_sub"])
        self._torque_pub = ZMQPublisher(zmq_addresses["joint_torque_sub"])
        self._raw_torque_pub = ZMQPublisher(zmq_addresses["raw_joint_torque_sub"])

        self._cmd_timer = self.create_timer(command_period_sec, self._forward_command)
        self.get_logger().info(
            f"Waiting for leader commands on {self._cmd_addr}, "
            f"robot state on {robot_state_topic}, "
            f"forwarding to {trajectory_topic} ..."
        )

    def _on_robot_state(self, msg: FrankaRobotState) -> None:
        q = np.array(msg.q[: self._num_arm_joints], dtype=np.float64)
        tau_ext = self._torque_sign * np.array(
            msg.tau_ext_hat_filtered[: self._num_arm_joints], dtype=np.float64
        )
        self.ee_pos = msg.o_t_ee[-4:-1]
        self._state_pub.send_message(q)
        self._torque_pub.send_message(tau_ext)
        self._raw_torque_pub.send_message(tau_ext)

    def out_of_bounds(self):
        # check x_direction
        if self.ee_pos[0] > 0.3 or self.ee_pos[0] < -0.3:
            return True
        # check y_direction
        elif self.ee_pos[1] > 0.4 or self.ee_pos[1] < -0.65:
            return True
        # check z_direction
        elif self.ee_pos[2] > 0.3 or self.ee_pos[1] < -0.6:
            return True
        # within bounds
        else:
            return False

    def _forward_command(self) -> None:
        arm_cmd = self._cmd_sub.message
        if arm_cmd is None:
            return
        if self.out_of_bounds():
            return
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in arm_cmd[: self._num_arm_joints]]
        point.time_from_start = self._trajectory_point_duration
        msg.points = [point]
        self._trajectory_pub.publish(msg)


def main(
    zmq_addresses: Dict[str, str],
    node_name: str = "factr_franka_ros2_follower",
    **kwargs,
) -> None:
    rclpy.init()
    follower = FrankaRos2Follower(
        zmq_addresses=zmq_addresses, node_name=node_name, **kwargs
    )
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()
