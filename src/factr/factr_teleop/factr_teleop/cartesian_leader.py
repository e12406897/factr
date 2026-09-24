"""Bridges a Cartesian-space input device (SpaceMouse, Omega.6, ...) to a FACTR follower
by publishing joint targets on the same `joint_pos_cmd_pub` ZMQ channel FACTRTeleop uses
(franka_ros2 v0.1.0 has no Cartesian controller, so IK has to happen leader-side).

The IK step is a copy of robosuite 1.5.2's IK_POSE controller
(robosuite/controllers/parts/arm/ik.py: _clip_ik_input + compute_joint_positions),
evaluated on robosuite's own Panda + PandaGripper MuJoCo model, with the follower's live
joint state standing in for robosuite's `sim.data.qpos`.
"""
import os
import time
from abc import ABC, abstractmethod
from typing import Tuple

import mujoco
import numpy as np
import rclpy
import robosuite.utils.transform_utils as T
import yaml
from python_utils.utils import get_workspace_root
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _build_robosuite_panda():
    from robosuite.models import MujocoWorldBase
    from robosuite.models.grippers import gripper_factory
    from robosuite.models.robots import Panda

    robot = Panda()
    # Same naming robosuite's Robot class uses, so the grip site is "gripper0_right_grip_site".
    gripper = gripper_factory("PandaGripper", idn="0_right")
    robot.add_gripper(gripper)
    world = MujocoWorldBase()
    world.merge(robot)
    model = world.get_model(mode="mujoco")
    return model, robot.joints, gripper.important_sites["grip_site"]


class CartesianLeader(Node, ABC):
    # robosuite IK_POSE defaults (ik.py compute_joint_positions / ik_pose.json)
    KN = np.array([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])
    DAMPING_PSEUDO_INV = 0.05
    KPOS = 0.95
    KORI = 0.95
    INTEGRATION_DT = 0.1
    MAX_ANGVEL = 1.0  # velocity_limits=[-1, 1] hardcoded in IK_POSE.get_control

    def __init__(
        self,
        name: str,
        zmq_addresses: dict,
        num_arm_joints: int = 7,
        control_freq: float = 20.0,  # robosuite's default policy/control_freq
        ik_pos_limit: float = 0.02,
        ik_ori_limit: float = 0.05,
        node_name: str = "cartesian_leader",
    ):
        super().__init__(node_name)
        self.name = name
        self._num_arm_joints = num_arm_joints
        self._ik_pos_limit = ik_pos_limit
        self._ik_ori_limit = ik_ori_limit

        self._mj_model, joint_names, site_name = _build_robosuite_panda()
        self._mj_data = mujoco.MjData(self._mj_model)
        self._qpos_ids = [self._mj_model.joint(j).qposadr[0] for j in joint_names]
        self._dof_ids = [self._mj_model.joint(j).dofadr[0] for j in joint_names]
        self._site_id = self._mj_model.site(site_name).id

        config_path = os.path.join(
            get_workspace_root(),
            f"src/factr/factr_teleop/factr_teleop/configs/franka_{name}.yaml",
        )
        with open(config_path, "r") as f:
            self._gripper_open_cmd = float(
                yaml.safe_load(f)["gripper_teleop"]["actuation_range"]
            )

        self._state_sub = ZMQSubscriber(zmq_addresses["joint_state_sub"])
        self.get_logger().info(
            f"CartesianLeader '{name}': waiting for the follower's joint state on "
            f"{zmq_addresses['joint_state_sub']} ..."
        )
        while self._state_sub.message is None:
            time.sleep(0.1)
        # Null-space posture target (robosuite's `initial_joint`): where the follower starts.
        self._q0 = self._follower_q()

        self._cmd_pub = ZMQPublisher(zmq_addresses["joint_pos_cmd_pub"])
        self._gripper_pub = self.create_publisher(
            JointState, f"/factr_teleop/{name}/cmd_gripper_pos", 10
        )

        self.get_logger().info(
            f"CartesianLeader '{name}' ready, publishing on "
            f"{zmq_addresses['joint_pos_cmd_pub']}. Start joints: {self._q0}"
        )
        self._timer = self.create_timer(1.0 / control_freq, self._control_loop_callback)

    @abstractmethod
    def read_device(self) -> Tuple[np.ndarray, np.ndarray, bool, bool]:
        """Called once per control tick. `self.ee_pos` / `self.ee_rot` hold the follower's
        current grip-site pose (base frame) at that point, for absolute-pose devices.

        Returns:
            dpos: (3,) end-effector position delta for this step, world/base frame [m]
                (robosuite IK_POSE action[:3]; clipped to ik_pos_limit here).
            drot: (3,) axis-angle rotation delta for this step [rad]
                (robosuite IK_POSE action[3:]; clipped to ik_ori_limit here).
            grasp: True = gripper should be closed.
            should_stop: True to request shutdown.
        """
        raise NotImplementedError

    def _follower_q(self) -> np.ndarray:
        return np.array(self._state_sub.message[: self._num_arm_joints], dtype=np.float64)

    def _update_kinematics(self, q: np.ndarray) -> None:
        m, d = self._mj_model, self._mj_data
        d.qpos[self._qpos_ids] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        self.ee_pos = d.site_xpos[self._site_id].copy()
        self.ee_rot = d.site_xmat[self._site_id].reshape(3, 3).copy()

    def _compute_joint_positions(
        self, q: np.ndarray, dpos: np.ndarray, drot: np.ndarray
    ) -> np.ndarray:
        """Expects _update_kinematics(q) to have been called for this q."""
        # --- IK_POSE._clip_ik_input ---
        if dpos.any():
            dpos, _ = T.clip_translation(dpos, self._ik_pos_limit)
        quat = T.axisangle2quat(drot)
        quat, _ = T.clip_rotation(quat, self._ik_ori_limit)
        rot = T.quat2mat(quat)

        # --- IK_POSE.compute_joint_positions (single site, delta) ---
        m, d = self._mj_model, self._mj_data

        twist = np.zeros(6)
        error_quat = np.zeros(4)
        twist[:3] = self.KPOS * dpos / self.INTEGRATION_DT
        mujoco.mju_mat2Quat(error_quat, rot.reshape(-1))
        mujoco.mju_quat2Vel(twist[3:], error_quat, 1.0)
        twist[3:] *= self.KORI / self.INTEGRATION_DT

        jac = np.zeros((6, m.nv), dtype=np.float64)
        mujoco.mj_jacSite(m, d, jac[:3], jac[3:], self._site_id)
        jac = jac[:, self._dof_ids]

        diag = self.DAMPING_PSEUDO_INV**2 * np.eye(6)
        eye = np.eye(len(self._dof_ids))
        dq = jac.T @ np.linalg.solve(jac @ jac.T + diag, twist)
        dq += (eye - np.linalg.pinv(jac) @ jac) @ (self.KN * (self._q0 - q))

        dq_abs_max = np.abs(dq).max()
        if dq_abs_max > self.MAX_ANGVEL:
            dq *= self.MAX_ANGVEL / dq_abs_max

        return q + dq * self.INTEGRATION_DT

    def _control_loop_callback(self) -> None:
        q = self._follower_q()
        self._update_kinematics(q)
        dpos, drot, grasp, should_stop = self.read_device()
        if should_stop:
            self.get_logger().info(f"CartesianLeader '{self.name}': stop requested.")
            self._timer.cancel()
            return

        q_des = self._compute_joint_positions(
            q, np.asarray(dpos, dtype=np.float64), np.asarray(drot, dtype=np.float64)
        )
        self._cmd_pub.send_message(q_des)

        # Every tick (like FACTRTeleop); the followers only act on state changes.
        msg = JointState()
        msg.position = [0.0 if grasp else self._gripper_open_cmd]
        self._gripper_pub.publish(msg)


def spin(leader: CartesianLeader) -> None:
    try:
        rclpy.spin(leader)
    except KeyboardInterrupt:
        pass
    finally:
        leader.destroy_node()
        rclpy.shutdown()
