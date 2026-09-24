"""Bridges a Cartesian-space input device (SpaceMouse, Omega.6, ...) to a FACTR
follower. Unlike FACTRTeleop (built around a joint-space, force-feedback Dynamixel
exoskeleton), these devices report end-effector translation/rotation deltas, not joint
angles.

Design choice (see the conversation this came out of): inverse kinematics is solved
HERE, on the leader side, against the FACTR exoskeleton's own URDF -- which already
shares the follower's joint kinematics by construction (that's the whole point of a
1:1 replica leader arm) -- and the result is published on the SAME `joint_pos_cmd_pub`
ZMQ channel FACTRTeleop uses. Nothing on the follower side (real franka_ros2 bridge or
robosuite) or the ZMQ protocol needs to change, and both keep working unmodified. The
alternative (follower-side Cartesian impedance controller) was ruled out for real
hardware: franka_ros2 v0.1.0 (this repo's pinned tag) ships no Cartesian controller at
all -- only gravity_compensation/joint_impedance/model/move_to_start example
controllers -- so that path only exists in the robosuite path anyway.
"""
import os
from abc import ABC, abstractmethod
from typing import Optional, Tuple

import numpy as np
import pinocchio as pin
import rclpy
from python_utils.utils import get_workspace_root
from python_utils.zmq_messenger import ZMQPublisher
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CartesianLeader(Node, ABC):
    """Subclasses implement `read_device()`, called once per control tick, returning
    the device's translation/rotation deltas plus gripper/button state. This class
    integrates those deltas into a target end-effector pose, solves damped-least-
    -squares differential IK (Pinocchio's own standard CLIK recipe) for the
    corresponding joint target, and publishes it -- same responsibility as
    FACTRTeleop.control_loop_callback(), just without any of the force-feedback /
    torque control machinery a passive input device has no use for.
    """

    def __init__(
        self,
        name: str,
        zmq_addresses: dict,
        urdf_path: str = "src/factr/factr_teleop/factr_teleop/urdf/factr_teleop_franka.urdf",
        num_arm_joints: int = 7,
        control_freq: float = 100.0,
        home_joint_pos: Optional[np.ndarray] = None,
        translation_scale: float = 1.0,
        rotation_scale: float = 1.0,
        # Cartesian workspace clamp (meters), safety net independent of the follower's
        # own out_of_bounds() check -- keeps the IK target (and thus the commanded
        # joint solution) from running away if a device reports a large spurious delta.
        workspace_min: Tuple[float, float, float] = (-0.6, -0.6, 0.0),
        workspace_max: Tuple[float, float, float] = (0.6, 0.6, 1.0),
        gripper_actuation_range: float = 0.08,
        ik_damping: float = 1e-2,
        ik_gain: float = 1.0,
        node_name: str = "cartesian_leader",
    ):
        super().__init__(node_name)
        self.name = name
        self._num_arm_joints = num_arm_joints
        self._translation_scale = translation_scale
        self._rotation_scale = rotation_scale
        self._workspace_min = np.array(workspace_min)
        self._workspace_max = np.array(workspace_max)
        self._gripper_actuation_range = gripper_actuation_range
        self._ik_damping = ik_damping
        self._ik_gain = ik_gain
        self._dt = 1.0 / control_freq

        workspace_root = get_workspace_root()
        full_urdf_path = os.path.join(workspace_root, urdf_path)
        urdf_dir = os.path.dirname(full_urdf_path)
        self._pin_model, _, _ = pin.buildModelsFromUrdf(
            filename=full_urdf_path, package_dirs=urdf_dir
        )
        self._pin_data = self._pin_model.createData()
        # Pinocchio joint index of the arm's tip (== num_arm_joints: base_link is the
        # root, link_1..link_7 are joints 1..7 -- same convention factr_teleop.py's
        # `pin.computeJointJacobian(..., self.num_arm_joints)` already relies on).
        self._tip_joint_id = num_arm_joints

        self.q = (
            np.array(home_joint_pos, dtype=float)
            if home_joint_pos is not None
            else np.zeros(num_arm_joints)
        )
        pin.forwardKinematics(self._pin_model, self._pin_data, self.q)
        home_pose = self._pin_data.oMi[self._tip_joint_id]
        self._target_pos = home_pose.translation.copy()
        self._target_rot = home_pose.rotation.copy()

        self._cmd_pub = ZMQPublisher(zmq_addresses["joint_pos_cmd_pub"])
        self._gripper_pub = self.create_publisher(
            JointState, f"/factr_teleop/{name}/cmd_gripper_pos", 10
        )
        self._gripper_closed = False

        self.get_logger().info(
            f"CartesianLeader '{name}' ready, publishing on "
            f"{zmq_addresses['joint_pos_cmd_pub']}. Home pose: "
            f"pos={self._target_pos}, joints={self.q}."
        )
        self._timer = self.create_timer(self._dt, self._control_loop_callback)

    @abstractmethod
    def read_device(self) -> Tuple[np.ndarray, np.ndarray, bool, bool]:
        """Called once per control tick.

        Returns:
            dpos (np.ndarray, shape (3,)): translation delta this tick, device units
                (get scaled by translation_scale -- device-specific normalization, if
                any, happens in the subclass).
            drot (np.ndarray, shape (3,)): rotation delta this tick as an axis-angle
                (rotation) vector.
            gripper_toggle (bool): True exactly on the tick the operator asked to
                open/close the gripper (e.g. a button *edge*, not "is held down" --
                debouncing is the subclass's responsibility, matching the binary
                open/close gripper convention FrankaRos2Follower/RobosuiteFrankaFollower
                already use).
            should_stop (bool): True to request shutdown (e.g. a dedicated device
                button), checked every tick.
        """
        raise NotImplementedError

    def _solve_ik_step(self) -> None:
        """One damped-least-squares CLIK step toward (self._target_pos,
        self._target_rot) -- Pinocchio's own standard inverse-kinematics recipe (see
        their "Inverse kinematics" example): err = log6(current^-1 * desired), solved
        in the LOCAL joint frame (matches computeJointJacobian's default reference
        frame), damped for robustness near singularities, then integrated on the
        manifold (not a naive q += dq, which breaks for e.g. continuous joints)."""
        pin.forwardKinematics(self._pin_model, self._pin_data, self.q)
        current = self._pin_data.oMi[self._tip_joint_id]
        desired = pin.SE3(self._target_rot, self._target_pos)
        err = pin.log6(current.inverse() * desired).vector

        J = pin.computeJointJacobian(
            self._pin_model, self._pin_data, self.q, self._tip_joint_id
        )
        damp = self._ik_damping**2 * np.eye(6)
        dq = J.T @ np.linalg.solve(J @ J.T + damp, err)
        self.q = pin.integrate(self._pin_model, self.q, dq * self._ik_gain)

    def _control_loop_callback(self) -> None:
        dpos, drot, gripper_toggle, should_stop = self.read_device()
        if should_stop:
            self.get_logger().info(f"CartesianLeader '{self.name}': stop requested.")
            self._timer.cancel()
            return

        self._target_pos = np.clip(
            self._target_pos + dpos * self._translation_scale,
            self._workspace_min,
            self._workspace_max,
        )
        if np.linalg.norm(drot) > 1e-9:
            self._target_rot = (
                pin.exp3(drot * self._rotation_scale) @ self._target_rot
            )

        self._solve_ik_step()
        self._cmd_pub.send_message(self.q[: self._num_arm_joints])

        if gripper_toggle:
            self._gripper_closed = not self._gripper_closed
            width = 0.0 if self._gripper_closed else self._gripper_actuation_range
            msg = JointState()
            msg.position = [float(width)]
            self._gripper_pub.publish(msg)


def spin(leader: CartesianLeader) -> None:
    try:
        rclpy.spin(leader)
    except KeyboardInterrupt:
        pass
    finally:
        leader.destroy_node()
        rclpy.shutdown()
