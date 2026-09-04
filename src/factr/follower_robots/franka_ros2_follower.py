from typing import Dict, List, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from franka_msgs.msg import FrankaRobotState
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
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

    Gripper (ROS, not ZMQ — matches FACTRTeleopFrankaZMQ.set_up_communication()):
      - subscribes to `/factr_teleop/{name}/cmd_gripper_pos`: leader gripper position
      - publishes  on `/gripper/{name}/obs_gripper_torque`:  gripper force feedback
      Uses `franka_gripper_node`'s `control_msgs/action/GripperCommand` action server.
      Actions are goal/result oriented, not built for per-tick streaming (unlike the
      arm's trajectory topic) — sending a new goal at 500Hz would likely get most goals
      rejected (no preemption support to count on) and load the DDS layer for no
      physical benefit, since the gripper mechanism can't track that fast anyway. So a
      new goal is only sent when the target changes by more than
      `gripper_goal_position_threshold`, or at least every
      `gripper_goal_refresh_period_sec` as a keep-alive so feedback (which only flows
      while a goal is active) doesn't go stale. The radians-to-width conversion
      (`gripper_actuation_range` -> `gripper_width_max`) assumes a linear mapping with
      0 = closed — verify against your actual leader gripper convention and adjust.

    Trajectory timing: `trajectory_point_duration_sec` is the MINIMUM time given to
    `joint_trajectory_controller` to reach each new target — used as-is while the
    target stays within `joint_distance_threshold` (rad, the largest single-joint
    move) of the last known robot position. Beyond that threshold, the duration is
    scaled up proportionally to how far the target distance exceeds the threshold,
    so a sudden large jump (e.g. leader command loss/resync) doesn't get commanded at
    the same speed as a small tracking correction and trip a Franka velocity/reflex
    fault.

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
    """

    JOINT_NAMES: List[str] = [f"panda_joint{i}" for i in range(1, 8)]

    def __init__(
        self,
        zmq_addresses: Dict[str, str],
        name: str = "left",
        num_arm_joints: int = 7,
        trajectory_topic: str = "/joint_trajectory_controller/joint_trajectory",
        robot_state_topic: str = "/franka_robot_state_broadcaster/robot_state",
        node_name: str = "factr_franka_ros2_follower",
        command_period_sec: float = 0.002,
        trajectory_point_duration_sec: float = 0.1,
        joint_distance_threshold: float = 0.5,
        torque_sign: float = 1.0,
        enable_gripper: bool = True,
        gripper_action_name: str = "/gripper_action",
        gripper_actuation_range: float = 0.8,
        gripper_width_max: float = 0.08,
        gripper_max_effort: float = 20.0,
        gripper_goal_position_threshold: float = 0.005,
        gripper_goal_refresh_period_sec: float = 0.1,
    ):
        super().__init__(node_name)
        self._num_arm_joints = num_arm_joints
        self._torque_sign = torque_sign
        self.ee_pos = np.zeros(3)

        self._min_trajectory_point_duration_sec = trajectory_point_duration_sec
        self._joint_distance_threshold = joint_distance_threshold
        self._current_q: Optional[np.ndarray] = None
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

        self._enable_gripper = enable_gripper
        if self._enable_gripper:
            self._gripper_actuation_range = gripper_actuation_range
            self._gripper_width_max = gripper_width_max
            self._gripper_max_effort = gripper_max_effort
            self._gripper_goal_position_threshold = gripper_goal_position_threshold
            self._gripper_last_goal_width: Optional[float] = None
            # gripper_state == 0 means CLOSED (width == 0, fingers touching), so
            # defaulting the target to 0 would command the gripper closed as soon as
            # this node starts — before the leader has sent anything, and before it's
            # matched to the leader's actual (open) trigger position. Default to fully
            # open instead, matching MujocoFrankaFollower's initial_gripper_cmd fix.
            self._gripper_target_width = gripper_width_max

            self._gripper_client = ActionClient(self, GripperCommand, gripper_action_name)
            self._gripper_cmd_sub = self.create_subscription(
                JointState,
                f"/factr_teleop/{name}/cmd_gripper_pos",
                self._on_gripper_cmd,
                10,
            )
            self._gripper_torque_pub = self.create_publisher(
                JointState, f"/gripper/{name}/obs_gripper_torque", 10
            )
            self._gripper_goal_timer = self.create_timer(
                gripper_goal_refresh_period_sec, self._maybe_send_gripper_goal
            )

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
        self.ee_pos = np.array(msg.o_t_ee[-4:-1], dtype=np.float64)
        self._current_q = q
        self._state_pub.send_message(q)
        self._torque_pub.send_message(tau_ext)
        self._raw_torque_pub.send_message(tau_ext)

    def out_of_bounds(self) -> bool:
        # check x_direction
        if self.ee_pos[0] > 0.65 or self.ee_pos[0] < 0.4:
            return True
        # check y_direction
        elif self.ee_pos[1] > 0.3 or self.ee_pos[1] < -0.3:
            return True
        # check z_direction
        elif self.ee_pos[2] > 0.6 or self.ee_pos[2] < 0.3:
            return True
        # within bounds
        else:
            return False

    def _trajectory_point_duration(self, target_q: np.ndarray) -> Duration:
        duration_sec = self._min_trajectory_point_duration_sec
        if self._current_q is not None:
            max_distance = float(np.max(np.abs(target_q - self._current_q)))
            if max_distance > self._joint_distance_threshold:
                duration_sec *= max_distance / self._joint_distance_threshold
        return Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9),
        )

    def _forward_command(self) -> None:
        arm_cmd = self._cmd_sub.message
        if arm_cmd is None:
            return
        if self.out_of_bounds():
            return
        target_q = np.array(arm_cmd[: self._num_arm_joints], dtype=np.float64)
        msg = JointTrajectory()
        msg.joint_names = self.JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in target_q]
        point.time_from_start = self._trajectory_point_duration(target_q)
        msg.points = [point]
        self._trajectory_pub.publish(msg)

    def _on_gripper_cmd(self, msg: JointState) -> None:
        leader_gripper_pos = float(msg.position[0])
        fraction = np.clip(leader_gripper_pos / self._gripper_actuation_range, 0.0, 1.0)
        self._gripper_target_width = float(fraction * self._gripper_width_max)

    def _maybe_send_gripper_goal(self) -> None:
        target = self._gripper_target_width
        if (
            self._gripper_last_goal_width is not None
            and abs(target - self._gripper_last_goal_width)
            < self._gripper_goal_position_threshold
        ):
            return
        # if not self._gripper_client.wait_for_server(timeout_sec=0.0):
        #     self.get_logger().warn(
        #         "Gripper action server not available yet, skipping goal "
        #         f"(target width={target:.4f})"
        #     )
        #     return
        self.get_logger().info(f"Sending gripper goal: width={target:.4f}")
        goal = GripperCommand.Goal()
        goal.command.position = target
        goal.command.max_effort = self._gripper_max_effort
        self._gripper_last_goal_width = target
        send_future = self._gripper_client.send_goal_async(
            goal, feedback_callback=self._on_gripper_feedback
        )
        send_future.add_done_callback(self._on_gripper_goal_response)

    def _on_gripper_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            return
        goal_handle.get_result_async()

    def _on_gripper_feedback(self, feedback_msg) -> None:
        effort = float(feedback_msg.feedback.effort)
        msg = JointState()
        msg.position = [effort]
        self._gripper_torque_pub.publish(msg)


def main(
    zmq_addresses: Dict[str, str],
    name: str = "left",
    node_name: str = "factr_franka_ros2_follower",
    **kwargs,
) -> None:
    rclpy.init()
    follower = FrankaRos2Follower(
        zmq_addresses=zmq_addresses, name=name, node_name=node_name, **kwargs
    )
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()
