import threading
import time
from typing import Dict, List

import mujoco
import numpy as np
import robosuite
from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber
from robosuite.controllers import load_composite_controller_config


class _GripperROSBridge:
    """Subscribes to each leader's `/factr_teleop/{name}/cmd_gripper_pos` ROS topic
    (matches `FACTRTeleopFrankaZMQ.set_up_communication()`) and forwards each into
    `follower.set_gripper_command(side, ...)`. Runs its own rclpy node/spin thread,
    same pattern as `sim_franka_follower.py`'s `GripperROSBridge`."""

    def __init__(self, follower: "RobosuiteFrankaFollower", names: List[str]):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        if not rclpy.ok():
            rclpy.init()
        self._follower = follower
        self._node = Node("robosuite_franka_gripper_bridge")
        for side, name in enumerate(names):
            self._node.create_subscription(
                JointState,
                f"/factr_teleop/{name}/cmd_gripper_pos",
                # Default arg binds `side` at definition time, not call time (closures
                # over a loop variable would otherwise all resolve to the last `side`).
                lambda msg, side=side: self._follower.set_gripper_command(
                    side, float(msg.position[0])
                ),
                1,
            )
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()


class RobosuiteFrankaFollower:
    """Bridges 1 or 2 FACTR ZMQ leaders to a robosuite environment with that many
    standard Panda arms + the default `PandaGripper`.

    - 1 leader: a single-arm env (e.g. `env_name="Lift"`), matching
      `MujocoFrankaFollower`/`mujoco_sim.py`'s per-side-process model -- launch one
      process per side with `--side left`/`--side right`.
    - 2 leaders: a single shared `TwoArm*` env (e.g. `env_name="TwoArmLift"`) driven by
      one physics step per control tick -- a real bimanual env needs one shared
      simulation, so both sides run in ONE process here, unlike the single-arm case.
    Robot index 0 (first entry of `robots=["Panda", ...]`) is driven by `names[0]`'s
    leader, index 1 (if present) by `names[1]`'s -- purely an ordering convention,
    unrelated to physical left/right placement `env_configuration` gives them in the
    scene.

    ZMQ wiring per side mirrors `MujocoFrankaFollower`/`FrankaRos2Follower`:
      - subscribes (connects) to `joint_pos_cmd_pub`: leader arm position targets
      - publishes  (binds)    on `joint_state_sub`:      follower arm joint positions
      - publishes  (binds)    on `joint_torque_sub`:     follower arm external joint
        torque, filtered (see `enable_var_scale_feedback` below)
      - publishes  (binds)    on `raw_joint_torque_sub`: same signal, unfiltered

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
    robosuite and this breaks). On top of the raw contact torque, the SAME adaptive
    smoothing filter as `MujocoFrankaFollower._get_arm_external_torque` is applied when
    `enable_var_scale_feedback=True`: fast tracking on rising edges (contact onset),
    slow decay otherwise, via a `tanh`-scaled step -- see `_filter_torque`. As in the
    MuJoCo follower, the filtered signal is sign-flipped before publishing on
    `joint_torque_sub` (empirically matched sign convention there; re-verify here).
    Gripper torque feedback is NOT implemented (the gripper's own
    `_ref_gripper_joint_vel_indexes` is keyed per-side internally and more fragile to
    rely on) -- only arm torque feedback is wired up.
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
        zmq_addresses: List[Dict[str, str]],
        gripper_actuation_range: List[float],
        names: List[str],
        env_name: str = "Lift",
        env_configuration: str = "default",
        num_arm_joints: int = 7,
        enable_ros_gripper: bool = True,
        has_renderer: bool = True,
        control_freq: int = 20,
        kp: float = 150.0,
        damping_ratio: float = 1.0,
        enable_var_scale_feedback: bool = False,
        var_scale_factor: float = 1.0,
    ):
        num_robots = len(zmq_addresses)
        assert num_robots in (1, 2), "RobosuiteFrankaFollower supports 1 or 2 robots."
        assert len(gripper_actuation_range) == num_robots
        assert len(names) == num_robots

        self._num_robots = num_robots
        self._num_arm_joints = num_arm_joints
        self._has_renderer = has_renderer
        self._control_period_sec = 1.0 / control_freq
        self._gripper_actuation_range = list(gripper_actuation_range)
        self._enable_var_scale_feedback = enable_var_scale_feedback
        self._var_scale_factor = var_scale_factor
        self._ext_arm_torque_prev = [np.zeros(num_arm_joints) for _ in range(num_robots)]

        arm_controller_config = {
            "type": "JOINT_POSITION",
            "input_type": "absolute",
            "kp": kp,
            "damping_ratio": damping_ratio,
            "impedance_mode": "fixed",
            "interpolation": None,
        }
        controller_configs = []
        for _ in range(num_robots):
            composite_config = load_composite_controller_config(robot="Panda")
            # A single-arm manipulator like Panda exposes exactly one arm part, keyed
            # "right" (robosuite's Panda.arms == ["right"]) regardless of the robot's
            # actual left/right placement in the scene -- NOT "right_arm" (that key
            # only exists on robosuite's unreleased master branch, not the 1.5.2
            # release on PyPI as of writing). Verify with
            # `load_composite_controller_config(robot="Panda")["body_parts"].keys()`
            # if this breaks again on a future robosuite version.
            # Merge into (not replace) the default "right" config -- it carries a
            # required nested "gripper" sub-config that a full overwrite would drop
            # (robot.py's _load_arm_controllers() asserts on "gripper" being present).
            composite_config["body_parts"]["right"].update(arm_controller_config)
            controller_configs.append(composite_config)

        make_kwargs = dict(
            env_name=env_name,
            robots=["Panda"] * num_robots,
            table_offset=(0, 0, 0.8)
            controller_configs=controller_configs,
            gripper_types="default",
            has_renderer=has_renderer,
            renderer="mjviewer",
            render_camera=None,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            control_freq=control_freq,
            ignore_done=True,
        )
        if num_robots == 2:
            # env_configuration (e.g. "opposed"/"parallel") is a TwoArmEnv-only kwarg --
            # single-arm envs like Lift don't accept it at all.
            make_kwargs["env_configuration"] = env_configuration
        self._env = robosuite.make(**make_kwargs)
        self._env.reset()

        if num_robots == 2:
            # Desired joint positions
            qpos_left = np.array([
                0.0, -0.5, 0.0, -2.0,
                0.0, 1.5, 0.785
            ])

            qpos_right = np.array([
                0.0, -0.5, 0.0, -2.0,
                0.0, 1.5, 0.785
            ])

            # Set robot joint positions
            self._env.robots[0].set_robot_joint_positions(qpos_left)
            self._env.robots[1].set_robot_joint_positions(qpos_right)

            
        else:
            qpos = np.array([
                            0.0, -0.5, 0.0, -2.0,
                            0.0, 1.5, 0.785
                        ])
            self._env.robots[0].set_robot_joint_positions(qpos)

        # Forward the simulation
        self._env.sim.forward()

        # Dense jacobian so `efc_J` (used by `_get_contact_torque`) comes back as a
        # plain (nefc, nv) array -- see MujocoFrankaFollower for the same setup.
        self._env.sim.model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE

        self._qpos_idx = [
            np.array(self._env.robots[i]._ref_joint_pos_indexes, dtype=int)
            for i in range(num_robots)
        ]
        self._dof_idx = [
            np.array(self._env.robots[i]._ref_joint_vel_indexes, dtype=int)
            for i in range(num_robots)
        ]

        self._cmd_sub = [ZMQSubscriber(a["joint_pos_cmd_pub"]) for a in zmq_addresses]
        self._state_pub = [ZMQPublisher(a["joint_state_sub"]) for a in zmq_addresses]
        self._torque_pub = [ZMQPublisher(a["joint_torque_sub"]) for a in zmq_addresses]
        self._raw_torque_pub = [
            ZMQPublisher(a["raw_joint_torque_sub"]) for a in zmq_addresses
        ]

        # Default gripper target to fully open (robosuite -1) so a leader that hasn't
        # sent anything yet doesn't get commanded closed at start -- same rationale as
        # MujocoFrankaFollower/FrankaRos2Follower's initial-gripper-open default.
        self._gripper_action = [self.GRIPPER_OPEN_ACTION] * num_robots

        self._gripper_bridge = (
            _GripperROSBridge(self, names) if enable_ros_gripper else None
        )

    def set_gripper_command(self, side: int, leader_gripper_pos: float) -> None:
        """`side`: index into `names`/`zmq_addresses` as passed to `__init__`. Called
        by the ROS gripper-command subscriber."""
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

    def _filter_torque(self, side: int, curr_ext_torque: np.ndarray) -> np.ndarray:
        """Adaptive smoothing identical to `MujocoFrankaFollower._get_arm_external_torque`:
        tracks rising edges (contact onset) almost immediately via a `tanh`-scaled
        step, but decays slowly otherwise, so brief contact spikes are felt promptly
        while noise gets smoothed out."""
        prev = self._ext_arm_torque_prev[side]
        if self._enable_var_scale_feedback:
            for i in range(len(curr_ext_torque)):
                delta = curr_ext_torque[i] - prev[i]
                if abs(curr_ext_torque[i]) - abs(prev[i]) > 0:
                    scale = np.tanh(self._var_scale_factor * abs(delta)) / (
                        self._var_scale_factor * abs(delta) + 1e-8
                    )
                    prev[i] += delta * scale
                else:
                    prev[i] = curr_ext_torque[i]
        else:
            prev = curr_ext_torque
        self._ext_arm_torque_prev[side] = prev
        return prev

    def _build_action(self) -> np.ndarray:
        action_parts = []
        for side in range(self._num_robots):
            arm_cmd = self._cmd_sub[side].message
            if arm_cmd is None:
                # Leader for this side hasn't sent anything yet -- hold the robot's
                # current position instead of stalling the whole shared env step (the
                # other side, if any, may already be teleoperating).
                arm_cmd = self._env.sim.data.qpos[self._qpos_idx[side]]
            action_parts.append(np.asarray(arm_cmd[: self._num_arm_joints], dtype=np.float64))
            action_parts.append(np.array([self._gripper_action[side]], dtype=np.float64))
        return np.concatenate(action_parts)

    def serve(self) -> None:
        print("Robosuite follower ready. Waiting for leader commands ...")
        while True:
            step_start = time.time()

            action = self._build_action()
            self._env.step(action)
            if self._has_renderer:
                self._env.render()

            for side in range(self._num_robots):
                q = self._env.sim.data.qpos[self._qpos_idx[side]].copy()
                raw_tau = self._get_contact_torque(self._dof_idx[side])
                filtered_tau = self._filter_torque(side, raw_tau)
                self._state_pub[side].send_message(q)
                self._torque_pub[side].send_message(-filtered_tau)
                self._raw_torque_pub[side].send_message(raw_tau)

            elapsed = time.time() - step_start
            if elapsed < self._control_period_sec:
                time.sleep(self._control_period_sec - elapsed)
