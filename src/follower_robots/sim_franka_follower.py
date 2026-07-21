import threading
import time
from typing import Dict, Optional

import mujoco
import mujoco.viewer
import numpy as np

from python_utils.zmq_messenger import ZMQPublisher, ZMQSubscriber

from dm_control import mjcf

assert mujoco.viewer is mujoco.viewer


def attach_hand_to_arm(
    arm_mjcf: mjcf.RootElement,
    hand_mjcf: mjcf.RootElement,
) -> None:
    """Attaches a hand to an arm.

    The arm must have a site named "attachment_site".

    Taken from https://github.com/deepmind/mujoco_menagerie/blob/main/FAQ.md#how-do-i-attach-a-hand-to-an-arm

    Args:
      arm_mjcf: The mjcf.RootElement of the arm.
      hand_mjcf: The mjcf.RootElement of the hand.

    Raises:
      ValueError: If the arm does not have a site named "attachment_site".
    """
    physics = mjcf.Physics.from_mjcf_model(hand_mjcf)

    attachment_site = arm_mjcf.find("site", "attachment_site")
    if attachment_site is None:
        raise ValueError("No attachment site found in the arm model.")

    # Expand the ctrl and qpos keyframes to account for the new hand DoFs.
    arm_key = arm_mjcf.find("key", "home")
    if arm_key is not None:
        hand_key = hand_mjcf.find("key", "home")
        if hand_key is None:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, np.zeros(physics.model.nu)])
            arm_key.qpos = np.concatenate([arm_key.qpos, np.zeros(physics.model.nq)])
        else:
            arm_key.ctrl = np.concatenate([arm_key.ctrl, hand_key.ctrl])
            arm_key.qpos = np.concatenate([arm_key.qpos, hand_key.qpos])

    attachment_site.attach(hand_mjcf)


def build_scene(robot_xml_path: str, gripper_xml_path: Optional[str] = None):
    # assert robot_xml_path.endswith(".xml")

    arena = mjcf.RootElement()
    arm_simulate = mjcf.from_path(robot_xml_path)
    # arm_copy = mjcf.from_path(xml_path)

    if gripper_xml_path is not None:
        # attach gripper to the robot at "attachment_site"
        gripper_simulate = mjcf.from_path(gripper_xml_path)
        attach_hand_to_arm(arm_simulate, gripper_simulate)

    # # Add wrist camera to the end effector
    # attachment_site = arm_simulate.find("site", "attachment_site")
    # if attachment_site is not None:
    #     # Add camera at the attachment site (wrist)
    #     attachment_site.parent.add(
    #         "camera",
    #         name="wrist_camera",
    #         pos=[0, 0, 0.05],  # 5cm above the attachment site
    #         quat=[0.707, 0.707, 0, 0],  # Looking forward and down
    #         fovy=60,  # Field of view
    #     )

    # Floor
    arena.worldbody.add(
        "geom",
        type="plane",
        size=[2, 2, 0.1],
        rgba=[0.5, 0.5, 0.5, 1],
        name="floor",
    )

    # Table: top surface at z=0.4, legs omitted for simplicity
    table_x, table_y, table_z = 0.5, 0.0, 0.4  # table top centre
    table_hw, table_hd, table_ht = 0.3, 0.3, 0.02  # half width/depth/thickness
    table_body = arena.worldbody.add(
        "body", name="table", pos=[table_x, table_y, table_z]
    )
    table_body.add(
        "geom",
        type="box",
        size=[table_hw, table_hd, table_ht],
        rgba=[0.6, 0.4, 0.2, 1],
        name="table_top",
    )

    # Hole: square socket flush on the table surface
    # 4 walls forming a square recess; peg must fit inside
    _hole_hw = 0.025  # inner half-width of the hole opening
    _wall_t = 0.005  # wall half-thickness
    _wall_h = 0.04  # socket depth half-height
    _wall_z = table_ht + _wall_h  # above table top
    hole_rgba = [0.3, 0.3, 0.8, 1]
    hole_body = arena.worldbody.add(
        "body", name="hole", pos=[table_x, table_y, table_z]
    )
    for name, pos, size in [
        (
            "hole_wall_n",
            [0, _hole_hw + _wall_t, _wall_z],
            [_hole_hw + _wall_t, _wall_t, _wall_h],
        ),
        (
            "hole_wall_s",
            [0, -(_hole_hw + _wall_t), _wall_z],
            [_hole_hw + _wall_t, _wall_t, _wall_h],
        ),
        (
            "hole_wall_e",
            [_hole_hw + _wall_t, 0, _wall_z],
            [_wall_t, _hole_hw + _wall_t, _wall_h],
        ),
        (
            "hole_wall_w",
            [-(_hole_hw + _wall_t), 0, _wall_z],
            [_wall_t, _hole_hw + _wall_t, _wall_h],
        ),
    ]:
        hole_body.add("geom", type="box", name=name, pos=pos, size=size, rgba=hole_rgba)

    # Peg: square box, starts on the table next to the hole
    peg_body = arena.worldbody.add(
        "body", name="peg", pos=[table_x - 0.15, table_y, table_z + table_ht + 0.04]
    )
    peg_body.add(
        "inertial", pos=[0, 0, 0], mass=0.1, diaginertia=[0.0001, 0.0001, 0.00005]
    )
    peg_body.add("joint", type="free", name="peg_joint")
    peg_body.add(
        "geom",
        type="box",
        size=[0.02, 0.02, 0.04],  # 4x4cm cross-section, 8cm tall
        rgba=[0.8, 0.3, 0.3, 1],
        name="peg_geom",
    )

    # Add a wall to the left side of the robot
    arena.worldbody.add(
        "geom",
        type="box",
        size=[0.01, 1.0, 0.75],  # thin (2cm), wide (2m), tall (1.5m)
        pos=[-0.3, 0.0, 0.75],
        rgba=[0.8, 0.8, 0.8, 1],
        name="wall",
    )

    arm_frame = arena.worldbody.attach(arm_simulate)
    # arena.worldbody.attach(arm_copy)

    return arena

class GripperROSBridge:
    """Optional rclpy side-car that lets the sim follower participate in FACTR's
    ROS-based gripper command/feedback topics, mirroring what the real Franka
    setup does via `franka_bridge.py`.
    """

    def __init__(self, follower: "MujocoFrankaFollower", name: str):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        self._JointState = JointState
        if not rclpy.ok():
            rclpy.init()
        self._follower = follower
        self._node = Node(f"mujoco_gripper_bridge_{name}")
        self._node.create_subscription(
            JointState, f"/factr_teleop/{name}/cmd_gripper_pos", self._cmd_callback, 1
        )
        self._torque_pub = self._node.create_publisher(
            JointState, f"/gripper/{name}/obs_gripper_torque", 1
        )
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._thread.start()

    def _cmd_callback(self, msg) -> None:
        self._follower.set_gripper_command(msg.position[0])

    def publish_torque(self, torque: float) -> None:
        msg = self._JointState()
        msg.position = [float(torque)]
        self._torque_pub.publish(msg)


class MujocoFrankaFollower:
    """Makes the MuJoCo FR3 simulation act as a drop-in replacement for the real
    Franka follower expected by `FACTRTeleopFrankaZMQ`.

    Unlike `MujocoRobotServer` (GELLO-style pickle REQ/REP), this class speaks
    FACTR's raw-numpy ZMQ PUB/SUB protocol directly, so the leader teleop node
    can run completely unmodified against either the real robot or this sim.

    ZMQ direction (mirrors the real Franka driver, which is external to this repo):
      - subscribes (connects) to `joint_pos_cmd_pub`: leader arm position targets
      - publishes  (binds)    on `joint_state_sub`:  follower arm joint positions
      - publishes  (binds)    on `joint_torque_sub`: follower arm external joint torque
    """

    ARM_JOINT_PREFIX = "fr3/fr3_joint"

    def __init__(
        self,
        xml_path: str,
        zmq_addresses: Dict[str, str],
        gripper_xml_path: Optional[str] = None,
        num_arm_joints: int = 7,
        print_joints: bool = False,
        enable_ros_gripper: bool = False,
        name: str = "sim",
        initial_arm_qpos: Optional[np.ndarray] = None,
    ):
        self._num_arm_joints = num_arm_joints
        self._has_gripper = gripper_xml_path is not None
        self._print_joints = print_joints

        arena = build_scene(xml_path, gripper_xml_path)
        assets: Dict[str, str] = {}
        for asset in arena.asset.all_children():
            if asset.tag == "mesh":
                assets[asset.file.get_vfs_filename()] = asset.file.contents
        self._model = mujoco.MjModel.from_xml_string(arena.to_xml_string(), assets)
        self._data = mujoco.MjData(self._model)

        self._resolve_arm_joint_indices()
        if initial_arm_qpos is not None:
            initial_arm_qpos = np.asarray(initial_arm_qpos, dtype=float)
            assert len(initial_arm_qpos) == self._num_arm_joints, (
                f"initial_arm_qpos must have length {self._num_arm_joints}, "
                f"got {len(initial_arm_qpos)}."
            )
            # Set both qpos and ctrl so the arm neither teleports nor sags toward the
            # default (zero) pose before the leader sends its first command.
            self._data.qpos[self._arm_qpos_adr] = initial_arm_qpos
            self._data.ctrl[: self._num_arm_joints] = initial_arm_qpos
            mujoco.mj_forward(self._model, self._data)
        self._gripper_ctrl_adr = self._num_arm_joints if self._has_gripper else None
        self._gripper_dof_adr = self._resolve_gripper_dof_index() if self._has_gripper else None
        self._gripper_cmd = 0.0

        self._cmd_addr = zmq_addresses["joint_pos_cmd_pub"]
        self._cmd_sub = ZMQSubscriber(self._cmd_addr)
        self._state_pub = ZMQPublisher(zmq_addresses["joint_state_sub"])
        self._torque_pub = ZMQPublisher(zmq_addresses["joint_torque_sub"])

        self._gripper_bridge = GripperROSBridge(self, name) if enable_ros_gripper else None
        self._stop_event = threading.Event()

    def _resolve_arm_joint_indices(self) -> None:
        qpos_adr = []
        dof_adr = []
        for i in range(1, self._num_arm_joints + 1):
            joint_name = f"{self.ARM_JOINT_PREFIX}{i}"
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise ValueError(f"Could not find joint '{joint_name}' in the MuJoCo model.")
            qpos_adr.append(self._model.jnt_qposadr[joint_id])
            dof_adr.append(self._model.jnt_dofadr[joint_id])
        self._arm_qpos_adr = np.array(qpos_adr, dtype=int)
        self._arm_dof_adr = np.array(dof_adr, dtype=int)

    def _resolve_gripper_dof_index(self) -> Optional[int]:
        # Panda hand joint naming depends on the attached menagerie asset. Try the
        # common candidates; fall back to no gripper torque feedback if none match.
        candidates = [
            "fr3/hand/finger_joint1",
            "fr3/finger_joint1",
            "hand/finger_joint1",
        ]
        for joint_name in candidates:
            joint_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id >= 0:
                return int(self._model.jnt_dofadr[joint_id])
        print(
            "MujocoFrankaFollower: could not resolve gripper joint name for torque "
            "feedback. Run list_joint_names() to inspect the model and update "
            "_resolve_gripper_dof_index()."
        )
        return None

    def list_joint_names(self) -> list:
        """Debug helper: print every joint name defined in the compiled model."""
        names = []
        for joint_id in range(self._model.njnt):
            name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            names.append(name)
            print(joint_id, name)
        return names

    def set_gripper_command(self, value: float) -> None:
        self._gripper_cmd = value

    def _get_arm_positions(self) -> np.ndarray:
        return self._data.qpos[self._arm_qpos_adr].copy()

    def _get_arm_velocities(self) -> np.ndarray:
        return self._data.qvel[self._arm_dof_adr].copy()

    def _get_arm_external_torque(self) -> np.ndarray:
        return self._data.qfrc_constraint[self._arm_dof_adr].copy()

    def _get_gripper_external_torque(self) -> float:
        if self._gripper_dof_adr is None:
            return 0.0
        return float(self._data.qfrc_constraint[self._gripper_dof_adr])

    def _apply_arm_command(self, arm_cmd: np.ndarray) -> None:
        assert len(arm_cmd) == self._num_arm_joints, (
            f"Expected arm command of length {self._num_arm_joints}, got {len(arm_cmd)}."
        )
        self._data.ctrl[: self._num_arm_joints] = arm_cmd

    def serve(self) -> None:
        print(f"MuJoCo FACTR follower ready. Waiting for leader commands on {self._cmd_addr} ...")
        with mujoco.viewer.launch_passive(self._model, self._data) as viewer:
            while viewer.is_running() and not self._stop_event.is_set():
                step_start = time.time()

                arm_cmd = self._cmd_sub.message
                if arm_cmd is not None:
                    self._apply_arm_command(arm_cmd[: self._num_arm_joints])
                if self._has_gripper:
                    self._data.ctrl[self._gripper_ctrl_adr] = self._gripper_cmd * 255

                mujoco.mj_step(self._model, self._data)
                self._state_pub.send_message(self._get_arm_positions())
                self._torque_pub.send_message(self._get_arm_external_torque())
                if self._gripper_bridge is not None:
                    self._gripper_bridge.publish_torque(self._get_gripper_external_torque())

                if self._print_joints:
                    print(self._get_arm_positions())

                viewer.sync()
                time_until_next_step = self._model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def stop(self) -> None:
        self._stop_event.set()
