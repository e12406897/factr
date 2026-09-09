from typing import Dict, List, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from franka_msgs.action import Grasp, Move
from franka_msgs.msg import FrankaRobotState
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import os
import yaml
from python_utils.utils import get_workspace_root


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
      Uses `franka_gripper_node`'s native `franka_msgs/action/Grasp` and
      `franka_msgs/action/Move` action servers directly (not the `control_msgs/GripperCommand`
      wrapper -- that wrapper always calls `franka::Gripper::grasp()` with a tight default
      epsilon tolerance (`default_grasp_epsilon`, ~5mm) around the *commanded* width; for an
      object whose actual width isn't known in advance, the resulting width almost always
      falls outside that tolerance, libfranka reports the grasp as failed, and the gripper
      does not keep applying holding force. Calling `Grasp` directly lets us pass a wide
      `epsilon` (`gripper_grasp_epsilon`, defaults to `gripper_width_max` -- i.e. accept any
      resulting width as a successful grasp) so force is held regardless of object size.

      Actions are goal/result oriented, not built for per-tick streaming (unlike the arm's
      trajectory topic), and continuous position tracking doesn't work well here anyway
      since `move()`/`grasp()` are slow, blocking, non-preemptible calls -- a second command
      while one is still executing throws and can release an already-applied grasp force
      (see `_gripper_goal_in_flight`). So the leader gripper signal is treated as a binary
      switch instead of a continuous position, with hysteresis around the midpoint of
      `gripper_actuation_range` (open above 60%, closed below 40%) to avoid chattering: below
      -> CLOSED (`Grasp`, width 0, force `gripper_max_effort`, speed `gripper_speed`, wide
      epsilon); above -> OPEN (`Move`, width `gripper_width_max`, speed `gripper_speed`). A
      new goal is only sent when the target state actually changes (see
      `gripper_goal_position_threshold`), not on every tick.

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
        save_launch: bool = False,
        num_arm_joints: int = 7,
        trajectory_topic: str = "/joint_trajectory_controller/joint_trajectory",
        robot_state_topic: str = "/franka_robot_state_broadcaster/robot_state",
        node_name: str = "factr_franka_ros2_follower",
        command_period_sec: float = 0.002,
        trajectory_point_duration_sec: float = 0.1,
        joint_distance_threshold: float = 0.5,
        torque_sign: float = 1.0,
        enable_gripper: bool = True,
        #define config file which is used bei factr teleoperation to get the same actuation range for the gripper
        config_file = 'franka_left.yaml',
        gripper_move_action_name: str = "/panda_gripper/move",
        gripper_grasp_action_name: str = "/panda_gripper/grasp",
        gripper_width_max: float = 0.075,
        # Franka Hand's max grasping force is ~70N; default to that for a firm "max force" grip.
        gripper_max_effort: float = 70.0,
        gripper_speed: float = 0.1,
        # How much the actual grasped width may deviate from the commanded width (0) and
        # still count as a successful grasp. None -> gripper_width_max, i.e. accept any
        # resulting width (we don't know the object size in advance) so libfranka always
        # keeps applying the holding force. See class docstring.
        gripper_grasp_epsilon: Optional[float] = None,
        gripper_goal_position_threshold: float = 0.01,
        gripper_goal_refresh_period_sec: float = 0.1,
        var_scale_factor: float = 1.0
    ):
        super().__init__(node_name)
        self.save_launch = save_launch
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
            config_path = os.path.join(
                get_workspace_root(),
                f"src/factr/factr_teleop/factr_teleop/configs/{config_file}",
            )
            with open(config_path, "r") as config_file:
                config = yaml.safe_load(config_file)
            self._gripper_actuation_range = config["gripper_teleop"]["actuation_range"]
            self._gripper_width_max = gripper_width_max
            self._gripper_max_effort = gripper_max_effort
            self._gripper_speed = gripper_speed
            self._gripper_grasp_epsilon = (
                gripper_grasp_epsilon if gripper_grasp_epsilon is not None else gripper_width_max
            )
            self._gripper_goal_position_threshold = gripper_goal_position_threshold
            self._gripper_last_goal_width: Optional[float] = None
            self._gripper_is_closed = False
            # franka_gripper_node does not allow overlapping gripper commands -- a second
            # goal while grasp()/move() is still executing throws and aborts the in-progress
            # one, which can release an already-applied grasp force. Track whether a goal is
            # still in flight and skip sending a new one until it completes.
            self._gripper_goal_in_flight = False
            # gripper_state == 0 means CLOSED (width == 0, fingers touching), so
            # defaulting the target to 0 would command the gripper closed as soon as
            # this node starts — before the leader has sent anything, and before it's
            # matched to the leader's actual (open) trigger position. Default to fully
            # open instead, matching MujocoFrankaFollower's initial_gripper_cmd fix.
            self._gripper_target_width = gripper_width_max

            self._gripper_move_client = ActionClient(self, Move, gripper_move_action_name)
            self._gripper_grasp_client = ActionClient(self, Grasp, gripper_grasp_action_name)
            self._gripper_cmd_sub = self.create_subscription(
                JointState,
                f"/factr_teleop/{name}/cmd_gripper_pos",
                self._on_gripper_cmd,
                10,
            )
            self._gripper_goal_timer = self.create_timer(
                gripper_goal_refresh_period_sec, self._maybe_send_gripper_goal
            )

        self.get_logger().info(
            f"Waiting for leader commands on {self._cmd_addr}, "
            f"robot state on {robot_state_topic}, "
            f"forwarding to {trajectory_topic} ..."
        )

        #filter cache
        self.ext_arm_torque_prev = np.zeros(num_arm_joints)
        self.var_scale_factor = var_scale_factor

    def _on_robot_state(self, msg: FrankaRobotState) -> None:
        q = np.array(msg.q[: self._num_arm_joints], dtype=np.float64)
        tau_ext = self._torque_sign * np.array(
            msg.tau_ext_hat_filtered[: self._num_arm_joints], dtype=np.float64
        )
        self.ee_pos = np.array(msg.o_t_ee[-4:-1], dtype=np.float64)
        self._current_q = q
        self._state_pub.send_message(q)
        tau_ext = self.filter_tau_ext(tau_ext)
        self._torque_pub.send_message(tau_ext)
        self._raw_torque_pub.send_message(tau_ext)

    def out_of_bounds(self) -> bool:
        if self.save_launch:
            print("bounding box enabled")
            # check x_direction
            if self.ee_pos[0] > 0.65 or self.ee_pos[0] < 0.3:
                return True
            # check y_direction
            elif self.ee_pos[1] > 0.3 or self.ee_pos[1] < -0.3:
                return True
            # check z_direction
            elif self.ee_pos[2] > 0.7 or self.ee_pos[2] < 0.3:
                return True
            # within bounds
            else:
                return False
        else:
            return False

    def filter_tau_ext(self, tau_ext):

        for i in range(len(tau_ext)):
            delta = tau_ext[i] - self.ext_arm_torque_prev[i]
            if abs(tau_ext[i]) - abs(self.ext_arm_torque_prev[i]) > 0:

                scale = np.tanh(self.var_scale_factor * abs(delta)) / (
                    self.var_scale_factor * abs(delta) + 1e-8
                )
                self.ext_arm_torque_prev[i] += delta * scale

            else:
                self.ext_arm_torque_prev[i] = tau_ext[i]

        return self.ext_arm_torque_prev


    def _trajectory_point_duration(self, target_q: np.ndarray) -> Duration:
        duration_sec = self._min_trajectory_point_duration_sec
        if self._current_q is not None:
            max_distance = float(np.max(np.abs(target_q - self._current_q)))
            if max_distance > self._joint_distance_threshold:
                duration_sec *= max_distance / 0.01
        return Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1.0) * 1e9),
        )

    def _forward_command(self) -> None:
        arm_cmd = self._cmd_sub.message
        if arm_cmd is None:
            return
        if self.out_of_bounds():
            target_q = self._current_q
        else:
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
        # Hysteresis around the midpoint so leader signal noise near the threshold (e.g.
        # while holding the trigger steady mid-squeeze) doesn't flip the target back and
        # forth -- each flip sends a new, unpreemptible gripper command that can interrupt
        # an in-progress grasp and release its holding force.
        close_threshold = self._gripper_actuation_range * 0.4
        open_threshold = self._gripper_actuation_range * 0.6
        if leader_gripper_pos < close_threshold:
            self._gripper_is_closed = True
        elif leader_gripper_pos > open_threshold:
            self._gripper_is_closed = False
        self._gripper_target_width = 0.0 if self._gripper_is_closed else self._gripper_width_max

    def _maybe_send_gripper_goal(self) -> None:
        if self._gripper_goal_in_flight:
            return
        target = self._gripper_target_width
        if (
            self._gripper_last_goal_width is not None
            and abs(target - self._gripper_last_goal_width)
            < self._gripper_goal_position_threshold
        ):
            return
        self._gripper_last_goal_width = target
        self._gripper_goal_in_flight = True
        if self._gripper_is_closed:
            self.get_logger().info(
                f"Sending gripper grasp: force={self._gripper_max_effort:.1f}N, "
                f"epsilon={self._gripper_grasp_epsilon:.4f}"
            )
            goal = Grasp.Goal()
            goal.width = 0.0
            goal.speed = self._gripper_speed
            goal.force = self._gripper_max_effort
            goal.epsilon.inner = self._gripper_grasp_epsilon
            goal.epsilon.outer = self._gripper_grasp_epsilon
            send_future = self._gripper_grasp_client.send_goal_async(goal)
        else:
            self.get_logger().info(f"Sending gripper move: width={target:.4f}")
            goal = Move.Goal()
            goal.width = target
            goal.speed = self._gripper_speed
            send_future = self._gripper_move_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_gripper_goal_response)

    def _on_gripper_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            self._gripper_goal_in_flight = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_gripper_result)

    def _on_gripper_result(self, future) -> None:
        self._gripper_goal_in_flight = False


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
