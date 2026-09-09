import threading
import time
from typing import Dict

import mujoco
import numpy as np
import robosuite
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from robosuite.controllers import load_composite_controller_config


class _GripperROSBridge:
    """Subscribes to both leaders' `/factr_teleop/{name}/cmd_gripper_pos` ROS topics
    (matches `FACTRTeleopFrankaZMQ.set_up_communication()`) and forwards each into
    `follower.set_gripper_command(side, ...)`. Runs its own rclpy node/spin thread,
    same pattern as `sim_franka_follower.py`'s `GripperROSBridge`."""

    def __init__(self, follower: "RobosuiteBimanualFollower", name_left: str, name_right: str):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init()
        self._follower = follower
        self._node = Node("robosuite_bimanual_gripper_bridge")
        self._node.create_subscription(
            JointState,
            f"/factr_teleop/{name_left}/cmd_gripper_pos",
            lambda msg: self._follower.set_gripper_command(0, float(msg.position[0])),
            1,
        )
        self._node.create_subscription(
            JointState,
            f"/factr_teleop/{name_right}/cmd_gripper_pos",
            lambda msg: self._follower.set_gripper_command(1, float(msg.position[0])),
            1,
        )
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()


class RobosuiteBimanualFollower:
    """Bridges TWO independent FACTR ZMQ leaders (`sim_left`, `sim_right`) to a single
    shared robosuite `TwoArm*` environment -- two standard Panda arms with the default
    `PandaGripper`, driven by one physics step per control tick (a real bimanual sim
    needs one shared env, unlike two independent single-arm `MujocoFrankaFollower`
    instances).

    ZMQ wiring per side mirrors `MujocoFrankaFollower`/`FrankaRos2Follower`:
      - subscribes (connects) to `joint_pos_cmd_pub`: leader arm position targets
      - publishes  (binds)    on `joint_state_sub`:      follower arm joint positions
      - publishes  (binds)    on `joint_torque_sub`:     follower arm external joint torque
      - publishes  (binds)    on `raw_joint_torque_sub`: same signal, unprocessed
    Robot index 0 (first entry of `robots=["Panda","Panda"]`) is driven by the LEFT
    leader, index 1 by the RIGHT leader -- purely a convention, unrelated to the
    physical left/right placement `env_configuration` gives them in the scene.

    Control: each robot's arm uses robosuite's `JOINT_POSITION` controller with
    `input_type="absolute"` (`goal_qpos = action` directly, no delta/scaling -- see
    `robosuite.controllers.parts.generic.joint_pos.JointPositionController.set_goal`),
    matching FACTR's leader, which always sends absolute joint targets. The gripper
    action is robosuite's standard convention: -1 = fully open, +1 = fully closed
    (verify empirically against your installed robosuite/gripper -- this is not
    guaranteed identical across robosuite versions).

    External-torque feedback reuses `MujocoFrankaFollower`'s contact-force-only
    filtering technique (excludes frictionloss/equality/limit constraints, keeps only
    genuine contacts) directly against robosuite's underlying `env.sim` -- robosuite
    wraps native MuJoCo, so `env.sim.model`/`env.sim.data` expose the same `efc_*`
    fields. DOF/qpos addresses per robot come from robosuite's own
    `env.robots[i]._ref_joint_pos_indexes` / `_ref_joint_vel_indexes` (internal,
    underscore-prefixed attributes -- re-verify these still exist if you upgrade
    robosuite and this breaks). Gripper torque feedback is NOT implemented (the
    gripper's own `_ref_gripper_joint_vel_indexes` is keyed per-side internally and
    more fragile to rely on) -- only arm torque feedback is wired up.
    """

    _CONTACT_CONSTRAINT_TYPES = (
        mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS,
        mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL,
        mujoco.mjtConstraint.mjCNSTR_CONTACT_ELLIPTIC,
        mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT,
        mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON,
        mujoco.mjtConstraint.mjCNSTR_EQUALITY,
    )
    GRIPPER_OPEN_ACTION = -1.0
    GRIPPER_CLOSE_ACTION = 1.0

    def __init__(
        self,
        zmq_addresses_left: Dict[str, str],
        zmq_addresses_right: Dict[str, str],
        gripper_actuation_range_left: float,
        gripper_actuation_range_right: float,
        name_left: str = "sim_left",
        name_right: str = "sim_right",
        env_name: str = "TwoArmLift",
        env_configuration: str = "default",
        num_arm_joints: int = 7,
        enable_ros_gripper: bool = True,
        has_renderer: bool = True,
        control_freq: int = 20,
        kp: float = 150.0,
        damping_ratio: float = 1.0,
    ):
        self._num_arm_joints = num_arm_joints
        self._has_renderer = has_renderer
        self._control_period_sec = 1.0 / control_freq
        self._gripper_actuation_range = [
            gripper_actuation_range_left,
            gripper_actuation_range_right,
        ]

        arm_controller_config = {
            "type": "JOINT_POSITION",
            "input_type": "absolute",
            "kp": kp,
            "damping_ratio": damping_ratio,
            "impedance_mode": "fixed",
            "interpolation": None,
        }
        controller_configs = []
        for _ in range(2):
            composite_config = load_composite_controller_config(robot="Panda")
            # Single-arm manipulators expose one arm part, keyed "right_arm" by
            # robosuite convention regardless of the robot's actual scene placement.
            composite_config["body_parts"]["right_arm"] = arm_controller_config
            controller_configs.append(composite_config)

        self._env = robosuite.make(
            env_name=env_name,
            robots=["Panda", "Panda"],
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            gripper_types="default",
            has_renderer=has_renderer,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            control_freq=control_freq,
            ignore_done=True,
        )
        self._env.reset()
        # Dense jacobian so `efc_J` (used by `_get_contact_torque`) comes back as a
        # plain (nefc, nv) array -- see MujocoFrankaFollower for the same setup.
        self._env.sim.model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE

        self._qpos_idx = [
            np.array(self._env.robots[i]._ref_joint_pos_indexes, dtype=int)
            for i in range(2)
        ]
        self._dof_idx = [
            np.array(self._env.robots[i]._ref_joint_vel_indexes, dtype=int)
            for i in range(2)
        ]

        addrs = [zmq_addresses_left, zmq_addresses_right]
        self._cmd_sub = [ZMQSubscriber(a["joint_pos_cmd_pub"]) for a in addrs]
        self._state_pub = [ZMQPublisher(a["joint_state_sub"]) for a in addrs]
        self._torque_pub = [ZMQPublisher(a["joint_torque_sub"]) for a in addrs]
        self._raw_torque_pub = [ZMQPublisher(a["raw_joint_torque_sub"]) for a in addrs]

        # Default gripper target to fully open (robosuite -1) so a leader that hasn't
        # sent anything yet doesn't get commanded closed at start -- same rationale as
        # MujocoFrankaFollower/FrankaRos2Follower's initial-gripper-open default.
        self._gripper_action = [self.GRIPPER_OPEN_ACTION, self.GRIPPER_OPEN_ACTION]

        self._gripper_bridge = (
            _GripperROSBridge(self, name_left, name_right) if enable_ros_gripper else None
        )

    def set_gripper_command(self, side: int, leader_gripper_pos: float) -> None:
        """`side`: 0 = left, 1 = right. Called by the ROS gripper-command subscriber."""
        fraction = np.clip(leader_gripper_pos / self._gripper_actuation_range[side], 0.0, 1.0)
        self._gripper_action[side] = (
            self.GRIPPER_OPEN_ACTION
            + fraction * (self.GRIPPER_CLOSE_ACTION - self.GRIPPER_OPEN_ACTION)
        )

    def _get_contact_torque(self, dof_adr: np.ndarray) -> np.ndarray:
        """Generalized force at the given DOFs from genuine contacts only (excludes
        frictionloss/equality/joint-limit constraints, which are internal to the
        model). See `MujocoFrankaFollower._get_contact_torque` for the source of this
        technique."""
        data = self._env.sim.data
        model = self._env.sim.model
        nefc = data.nefc
        if nefc == 0:
            return np.zeros(len(dof_adr))
        efc_J = data.efc_J.reshape(nefc, model.nv)
        is_contact = np.isin(data.efc_type[:nefc], self._CONTACT_CONSTRAINT_TYPES)
        return efc_J[is_contact][:, dof_adr].T @ data.efc_force[:nefc][is_contact]

    def _build_action(self) -> np.ndarray:
        action_parts = []
        for side in range(2):
            arm_cmd = self._cmd_sub[side].message
            if arm_cmd is None:
                # Leader for this side hasn't sent anything yet -- hold the robot's
                # current position instead of stalling the whole shared env step (the
                # other side may already be teleoperating).
                arm_cmd = self._env.sim.data.qpos[self._qpos_idx[side]]
            action_parts.append(np.asarray(arm_cmd[: self._num_arm_joints], dtype=np.float64))
            action_parts.append(np.array([self._gripper_action[side]], dtype=np.float64))
        return np.concatenate(action_parts)

    def serve(self) -> None:
        print("Robosuite bimanual follower ready. Waiting for leader commands ...")
        while True:
            step_start = time.time()

            action = self._build_action()
            self._env.step(action)
            if self._has_renderer:
                self._env.render()

            for side in range(2):
                q = self._env.sim.data.qpos[self._qpos_idx[side]].copy()
                tau_ext = self._get_contact_torque(self._dof_idx[side])
                self._state_pub[side].send_message(q)
                self._torque_pub[side].send_message(tau_ext)
                self._raw_torque_pub[side].send_message(tau_ext)

            elapsed = time.time() - step_start
            if elapsed < self._control_period_sec:
                time.sleep(self._control_period_sec - elapsed)
